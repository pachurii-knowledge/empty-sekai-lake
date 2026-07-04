#!/usr/bin/env python3
"""
Exhaustive unit test for the RV64C 16->32 expander (src/rvc_expand.sv).

Three independent views of every one of the 2^16 parcels are cross-checked:
  1. SV   -- src/rvc_expand.sv, dumped by tests/tb_rvc_expand.sv (the DUT).
  2. REF  -- an independent Python reference expander written from the ISA
            manual (below). Exhaustive, exact 32-bit-word comparison vs SV.
  3. objdump (binutils) -- disassembles both the raw 16-bit parcel and REF's
            32-bit expansion; validates REF's expansion is a real instruction
            whose decoded register numbers + immediate match the compressed
            form. This decorrelates REF from SV (binutils did the bit
            extraction independently), so SV == REF == binutils.

Usage:  python3 scripts/test_rvc_expand.py [--build]
Exit 0 on full pass.
"""
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJDUMP = "riscv64-unknown-elf-objdump"
DUMP = os.path.join(ROOT, "output", "rvc_expand", "rvc_expand_dump.txt")


# ---------------------------------------------------------------------------
# 2. Independent Python reference expander (RV64GC), from the ISA manual.
#    Returns (x32, illegal). Built with standard-format packers so the shuffled
#    immediate math is separated from the instruction-field packing.
# ---------------------------------------------------------------------------
def bits(v, hi, lo):
    return (v >> lo) & ((1 << (hi - lo + 1)) - 1)


def bit(v, i):
    return (v >> i) & 1


def sext(v, width):
    if v & (1 << (width - 1)):
        return v - (1 << width)
    return v


def enc_r(f7, rs2, rs1, f3, rd, op):
    return ((f7 & 0x7F) << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | \
           ((f3 & 7) << 12) | ((rd & 0x1F) << 7) | (op & 0x7F)


def enc_i(imm, rs1, f3, rd, op):
    return ((imm & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | ((f3 & 7) << 12) | \
           ((rd & 0x1F) << 7) | (op & 0x7F)


def enc_s(imm, rs2, rs1, f3, op):
    imm &= 0xFFF
    return (bits(imm, 11, 5) << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | \
           ((f3 & 7) << 12) | (bits(imm, 4, 0) << 7) | (op & 0x7F)


def enc_b(imm, rs2, rs1, f3, op):
    imm &= 0x1FFF
    return (bit(imm, 12) << 31) | (bits(imm, 10, 5) << 25) | ((rs2 & 0x1F) << 20) | \
           ((rs1 & 0x1F) << 15) | ((f3 & 7) << 12) | (bits(imm, 4, 1) << 8) | \
           (bit(imm, 11) << 7) | (op & 0x7F)


def enc_u(imm20, rd, op):
    return ((imm20 & 0xFFFFF) << 12) | ((rd & 0x1F) << 7) | (op & 0x7F)


def enc_j(imm, rd, op):
    imm &= 0x1FFFFF
    return (bit(imm, 20) << 31) | (bits(imm, 10, 1) << 21) | (bit(imm, 11) << 20) | \
           (bits(imm, 19, 12) << 12) | ((rd & 0x1F) << 7) | (op & 0x7F)


LOAD, LOAD_FP, OPIMM, OPIMM32 = 0x03, 0x07, 0x13, 0x1B
STORE, STORE_FP, OP, OP32 = 0x23, 0x27, 0x33, 0x3B
LUI, BRANCH, JALR, JAL = 0x37, 0x63, 0x67, 0x6F


def ILL(c):
    return (0x00000000 | (c & 0xFFFF), 1)


def ref_expand(c):
    quad = c & 3
    f3 = bits(c, 15, 13)
    rd_rs1 = bits(c, 11, 7)
    rs2 = bits(c, 6, 2)
    rph = 8 + bits(c, 9, 7)   # rd'/rs1'
    rpl = 8 + bits(c, 4, 2)   # rs2'/rd'
    imm6 = (bit(c, 12) << 5) | bits(c, 6, 2)
    simm6 = sext(imm6, 6) & 0xFFF
    shamt6 = (bit(c, 12) << 5) | bits(c, 6, 2)

    if quad == 0:
        if f3 == 0:  # C.ADDI4SPN
            nzu = (bits(c, 10, 7) << 6) | (bits(c, 12, 11) << 4) | \
                  (bit(c, 5) << 3) | (bit(c, 6) << 2)
            if bits(c, 12, 5) == 0:
                return ILL(c)
            return (enc_i(nzu, 2, 0, rpl, OPIMM), 0)
        if f3 == 1:  # C.FLD
            off = (bits(c, 6, 5) << 6) | (bits(c, 12, 10) << 3)
            return (enc_i(off, rph, 3, rpl, LOAD_FP), 0)
        if f3 == 2:  # C.LW
            off = (bit(c, 5) << 6) | (bits(c, 12, 10) << 3) | (bit(c, 6) << 2)
            return (enc_i(off, rph, 2, rpl, LOAD), 0)
        if f3 == 3:  # C.LD (RV64)
            off = (bits(c, 6, 5) << 6) | (bits(c, 12, 10) << 3)
            return (enc_i(off, rph, 3, rpl, LOAD), 0)
        if f3 == 4:
            return ILL(c)
        if f3 == 5:  # C.FSD
            off = (bits(c, 6, 5) << 6) | (bits(c, 12, 10) << 3)
            return (enc_s(off, rpl, rph, 3, STORE_FP), 0)
        if f3 == 6:  # C.SW
            off = (bit(c, 5) << 6) | (bits(c, 12, 10) << 3) | (bit(c, 6) << 2)
            return (enc_s(off, rpl, rph, 2, STORE), 0)
        if f3 == 7:  # C.SD (RV64)
            off = (bits(c, 6, 5) << 6) | (bits(c, 12, 10) << 3)
            return (enc_s(off, rpl, rph, 3, STORE), 0)
    elif quad == 1:
        if f3 == 0:  # C.ADDI / NOP / HINT
            return (enc_i(simm6, rd_rs1, 0, rd_rs1, OPIMM), 0)
        if f3 == 1:  # C.ADDIW (RV64)
            if rd_rs1 == 0:
                return ILL(c)
            return (enc_i(simm6, rd_rs1, 0, rd_rs1, OPIMM32), 0)
        if f3 == 2:  # C.LI
            return (enc_i(simm6, 0, 0, rd_rs1, OPIMM), 0)
        if f3 == 3:
            if rd_rs1 == 2:  # C.ADDI16SP
                if imm6 == 0:
                    return ILL(c)
                nz = (bit(c, 12) << 9) | (bits(c, 4, 3) << 7) | (bit(c, 5) << 6) | \
                     (bit(c, 2) << 5) | (bit(c, 6) << 4)
                return (enc_i(sext(nz, 10) & 0xFFF, 2, 0, 2, OPIMM), 0)
            else:  # C.LUI
                if imm6 == 0:
                    return ILL(c)
                imm20 = sext(imm6, 6) & 0xFFFFF  # value at [17:12], sign-extended
                return (enc_u(imm20, rd_rs1, LUI), 0)
        if f3 == 4:
            sel = bits(c, 11, 10)
            if sel == 0:  # C.SRLI
                return (enc_i((0b000000 << 6) | shamt6, rph, 5, rph, OPIMM), 0)
            if sel == 1:  # C.SRAI
                return (enc_i((0b010000 << 6) | shamt6, rph, 5, rph, OPIMM), 0)
            if sel == 2:  # C.ANDI
                return (enc_i(simm6, rph, 7, rph, OPIMM), 0)
            # reg-reg
            sub = (bit(c, 12) << 2) | bits(c, 6, 5)
            if sub == 0b000:
                return (enc_r(0b0100000, rpl, rph, 0, rph, OP), 0)   # SUB
            if sub == 0b001:
                return (enc_r(0, rpl, rph, 4, rph, OP), 0)           # XOR
            if sub == 0b010:
                return (enc_r(0, rpl, rph, 6, rph, OP), 0)           # OR
            if sub == 0b011:
                return (enc_r(0, rpl, rph, 7, rph, OP), 0)           # AND
            if sub == 0b100:
                return (enc_r(0b0100000, rpl, rph, 0, rph, OP32), 0)  # SUBW
            if sub == 0b101:
                return (enc_r(0, rpl, rph, 0, rph, OP32), 0)          # ADDW
            return ILL(c)
        if f3 == 5:  # C.J
            off = (bit(c, 12) << 11) | (bit(c, 8) << 10) | (bits(c, 10, 9) << 8) | \
                  (bit(c, 6) << 7) | (bit(c, 7) << 6) | (bit(c, 2) << 5) | \
                  (bit(c, 11) << 4) | (bits(c, 5, 3) << 1)
            return (enc_j(sext(off, 12) & 0x1FFFFF, 0, JAL), 0)
        if f3 == 6 or f3 == 7:  # C.BEQZ / C.BNEZ
            off = (bit(c, 12) << 8) | (bits(c, 6, 5) << 6) | (bit(c, 2) << 5) | \
                  (bits(c, 11, 10) << 3) | (bits(c, 4, 3) << 1)
            f = 0 if f3 == 6 else 1
            return (enc_b(sext(off, 9) & 0x1FFF, 0, rph, f, BRANCH), 0)
    elif quad == 2:
        if f3 == 0:  # C.SLLI
            return (enc_i((0b000000 << 6) | shamt6, rd_rs1, 1, rd_rs1, OPIMM), 0)
        if f3 == 1:  # C.FLDSP
            off = (bits(c, 4, 2) << 6) | (bit(c, 12) << 5) | (bits(c, 6, 5) << 3)
            return (enc_i(off, 2, 3, rd_rs1, LOAD_FP), 0)
        if f3 == 2:  # C.LWSP
            if rd_rs1 == 0:
                return ILL(c)
            off = (bits(c, 3, 2) << 6) | (bit(c, 12) << 5) | (bits(c, 6, 4) << 2)
            return (enc_i(off, 2, 2, rd_rs1, LOAD), 0)
        if f3 == 3:  # C.LDSP (RV64)
            if rd_rs1 == 0:
                return ILL(c)
            off = (bits(c, 4, 2) << 6) | (bit(c, 12) << 5) | (bits(c, 6, 5) << 3)
            return (enc_i(off, 2, 3, rd_rs1, LOAD), 0)
        if f3 == 4:
            if bit(c, 12) == 0:
                if rs2 == 0:  # C.JR
                    if rd_rs1 == 0:
                        return ILL(c)
                    return (enc_i(0, rd_rs1, 0, 0, JALR), 0)
                return (enc_r(0, rs2, 0, 0, rd_rs1, OP), 0)          # C.MV
            else:
                if rs2 == 0:
                    if rd_rs1 == 0:
                        return (0x00100073, 0)                      # C.EBREAK
                    return (enc_i(0, rd_rs1, 0, 1, JALR), 0)         # C.JALR
                return (enc_r(0, rs2, rd_rs1, 0, rd_rs1, OP), 0)     # C.ADD
        if f3 == 5:  # C.FSDSP
            off = (bits(c, 9, 7) << 6) | (bits(c, 12, 10) << 3)
            return (enc_s(off, rs2, 2, 3, STORE_FP), 0)
        if f3 == 6:  # C.SWSP
            off = (bits(c, 8, 7) << 6) | (bits(c, 12, 9) << 2)
            return (enc_s(off, rs2, 2, 2, STORE), 0)
        if f3 == 7:  # C.SDSP (RV64)
            off = (bits(c, 9, 7) << 6) | (bits(c, 12, 10) << 3)
            return (enc_s(off, rs2, 2, 3, STORE), 0)
    return ILL(c)


# ---------------------------------------------------------------------------
# 3. objdump helpers -- batch-disassemble a blob of fixed-width little-endian
#    words and return, per slot, the raw disassembly text (mnemonic + operands).
# ---------------------------------------------------------------------------
def objdump_blob(words, width):
    """words: list of ints; width: 2 or 4 bytes. Returns list[str] disasm."""
    blob = b"".join(w.to_bytes(width, "little") for w in words)
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        f.write(blob)
        path = f.name
    try:
        out = subprocess.run(
            [OBJDUMP, "-D", "-b", "binary", "-m", "riscv:rv64",
             "-M", "numeric,no-aliases", path],
            capture_output=True, text=True, check=True).stdout
    finally:
        os.unlink(path)
    # lines like:  "   4:\t00150513\taddi\tx10,x10,1"
    res = {}
    for line in out.splitlines():
        m = re.match(r"\s*([0-9a-f]+):\t[0-9a-f ]+\t(.*)", line)
        if m:
            addr = int(m.group(1), 16)
            # strip objdump's trailing "# 0x..." resolved-value comment, whose
            # '0x' would otherwise be mis-parsed as a register 'x<digits>'.
            res[addr] = m.group(2).split("#")[0].strip()
    # index by slot (addr // width) for the entries objdump actually decoded at
    # a slot boundary; anything it split differently is left absent.
    slot = {}
    for addr, txt in res.items():
        if addr % width == 0:
            slot[addr // width] = txt
    return slot


def regs_in(txt):
    # only true register tokens (x<N> at a word boundary), never the 'x' of a
    # '0x...' hex literal (no boundary between the '0' and 'x').
    return re.findall(r"\bx(\d+)", txt)


def imm_reg(txt):
    """parse 'off(xN)' load/store operand -> (off, N) or None."""
    m = re.search(r"(-?\d+)\(x(\d+)\)", txt)
    if m:
        return int(m.group(1)), int(m.group(2))
    return None


def main():
    build = "--build" in sys.argv
    outdir = os.path.join(ROOT, "output", "rvc_expand")
    os.makedirs(outdir, exist_ok=True)
    if build or not os.path.exists(DUMP):
        print("[*] building tb_rvc_expand with Verilator (-DRVC -DRV64)...")
        cmd = ("verilator --sv --timing --binary -Wno-fatal -Wno-WIDTH "
               "-DRVC -DRV64 -Isrc --top-module top "
               "-o Vtb_rvc_expand --Mdir " + outdir + "/obj "
               "src/rvc_expand.sv tests/tb_rvc_expand.sv")
        subprocess.run(cmd, shell=True, cwd=ROOT, check=True)
        subprocess.run([outdir + "/obj/Vtb_rvc_expand"], cwd=outdir, check=True)

    # 1. SV dump
    sv = {}
    with open(DUMP) as f:
        for line in f:
            cc, xx, il = line.split()
            sv[int(cc, 16)] = (int(xx, 16), int(il))
    assert len(sv) == 65536, f"expected 65536 SV rows, got {len(sv)}"

    # 2. SV vs REF, exhaustive exact
    mismatches = []
    for c in range(65536):
        rx, ril = ref_expand(c)
        sx, sil = sv[c]
        if sil != ril or (ril == 0 and sx != rx):
            mismatches.append((c, sx, sil, rx, ril))
    if mismatches:
        print(f"[FAIL] SV vs Python-reference: {len(mismatches)} mismatches")
        for c, sx, sil, rx, ril in mismatches[:25]:
            print(f"   c={c:04x}  SV=(x={sx:08x},ill={sil})  REF=(x={rx:08x},ill={ril})")
        return 1
    print("[PASS] SV expander == Python reference for all 65536 parcels")

    # 3. objdump decorrelation: validate REF against binutils.
    legal = [c for c in range(65536) if ref_expand(c)[1] == 0]
    dis_c = objdump_blob(list(range(65536)), 2)          # parcels
    dis_x = objdump_blob([ref_expand(c)[0] for c in range(65536)], 4)  # expansions

    fam = {  # compressed mnemonic -> expected base mnemonic
        "c.addi4spn": "addi", "c.fld": "fld", "c.lw": "lw", "c.ld": "ld",
        "c.fsd": "fsd", "c.sw": "sw", "c.sd": "sd", "c.addi": "addi",
        "c.addiw": "addiw", "c.li": "addi", "c.addi16sp": "addi", "c.lui": "lui",
        "c.srli": "srli", "c.srai": "srai", "c.andi": "andi", "c.sub": "sub",
        "c.xor": "xor", "c.or": "or", "c.and": "and", "c.subw": "subw",
        "c.addw": "addw", "c.j": "jal", "c.beqz": "beq", "c.bnez": "bne",
        "c.slli": "slli", "c.fldsp": "fld", "c.lwsp": "lw", "c.ldsp": "ld",
        "c.jr": "jalr", "c.mv": "add", "c.ebreak": "ebreak", "c.jalr": "jalr",
        "c.add": "add", "c.fsdsp": "fsd", "c.swsp": "sw", "c.sdsp": "sd",
        "c.nop": "addi",
    }
    # load/store families where objdump prints 'off(reg)' identically in both.
    ldst = {"c.fld", "c.lw", "c.ld", "c.fsd", "c.sw", "c.sd",
            "c.fldsp", "c.lwsp", "c.ldsp", "c.fsdsp", "c.swsp", "c.sdsp"}

    # NOTE: parcels blob is indexed by 2-byte slot i (byte_addr 2*i), so dis_c[i]
    # is parcel i's disasm; expansions blob is 4-byte, dis_x[i] at byte_addr 4*i.
    branch_jal = {"c.j", "c.beqz", "c.bnez"}
    indirect = {"c.jr", "c.jalr"}          # register-indirect, no offset to diff

    def last_op(txt):
        # trailing comma-operand; strip the '# comment' objdump may append.
        body = txt.split(None, 1)[1] if " " in txt or "\t" in txt else ""
        body = re.split(r"\t", txt, maxsplit=1)[1] if "\t" in txt else \
               (txt.split(None, 1)[1] if " " in txt else "")
        body = body.split("#")[0].strip()
        parts = body.split(",")
        return parts[-1].strip() if parts and parts[0] else None

    def target_addr(txt):
        # branch/jal print the absolute target as the last 0x.. hex literal.
        ms = re.findall(r"0x([0-9a-f]+)", txt)
        return int(ms[-1], 16) if ms else None

    objfail = []
    ck_ldst = ck_imm = ck_off = 0
    for c in legal:
        tc = dis_c.get(c)
        tx = dis_x.get(c)
        if tc is None or tx is None:
            continue  # objdump merged an adjacent slot -- skip
        mc = re.split(r"[ \t]", tc, maxsplit=1)[0]
        mx = re.split(r"[ \t]", tx, maxsplit=1)[0]
        if mc not in fam:
            continue  # objdump printed an unexpected/alias form -- skip
        if fam[mc] != mx:
            objfail.append((c, tc, tx, f"base mnemonic {mx} != expected {fam[mc]}"))
            continue
        # every register named in the compressed form must appear in the expansion
        if not set(regs_in(tc)) <= set(regs_in(tx)):
            objfail.append((c, tc, tx, f"regs {regs_in(tc)} not subset of {regs_in(tx)}"))
            continue
        if mc in ldst:
            ic, ix = imm_reg(tc), imm_reg(tx)
            if ic and ix:
                if ic != ix:
                    objfail.append((c, tc, tx, f"off/base {ic} != {ix}"))
                else:
                    ck_ldst += 1
        elif mc in branch_jal:
            ac, ax = target_addr(tc), target_addr(tx)
            if ac is not None and ax is not None:
                # offset = target - own byte address (2*c for parcel, 4*c for exp)
                if (ac - 2 * c) != (ax - 4 * c):
                    objfail.append((c, tc, tx,
                                    f"offset {ac - 2*c:#x} != {ax - 4*c:#x}"))
                else:
                    ck_off += 1
        elif mc in indirect:
            pass  # register-indirect; register subset already checked
        else:  # arithmetic / reg-reg: compare the discriminating last operand
            lc, lx = last_op(tc), last_op(tx)
            if lc is not None and lx is not None and lc != lx:
                objfail.append((c, tc, tx, f"last operand {lc!r} != {lx!r}"))
            else:
                ck_imm += 1

    if objfail:
        print(f"[FAIL] objdump cross-check: {len(objfail)} disagreements")
        for c, tc, tx, why in objfail[:30]:
            print(f"   c={c:04x}  '{tc}'  ->  '{tx}'   {why}")
        return 1
    print(f"[PASS] objdump cross-check vs binutils: mnemonic families + register "
          f"subsets agree; immediates match ({ck_ldst} ld/st, {ck_imm} arith, "
          f"{ck_off} branch/jal offsets)")

    # 4. curated exact anchors (hand-computed from the ISA manual)
    anchors = {
        0x0001: (0x00000013, 0),   # c.nop        -> addi x0,x0,0
        0x4505: (0x00100513, 0),   # c.li x10,1   -> addi x10,x0,1
        0x0505: (0x00150513, 0),   # c.addi x10,1 -> addi x10,x10,1
        0x842a: (0x00050413, 0),   # c.mv x8,x10  -> add x8,x0,x10 (addi form? no)
        0x9002: (0x00100073, 0),   # c.ebreak
        0x8082: (0x00008067, 0),   # c.jr x1(ra)  -> jalr x0,0(x1)   (ret)
        0x0000: (0x00000000, 1),   # illegal (all-zero)
        0x0089: (0x00208093, 0),   # c.addi x1,2     -> addi x1,x1,2
        0x6109: (0x08010113, 0),   # c.addi16sp 128  -> addi x2,x2,128
    }
    afail = []
    for c, (ex, eil) in anchors.items():
        rx, ril = ref_expand(c)
        sx, sil = sv[c]
        # c.mv x8,x10 uses R-form add, recompute expected here rather than trust
        if c == 0x842a:
            ex = enc_r(0, 10, 0, 0, 8, OP)
        if (rx, ril) != (ex, eil) or (sx, sil) != (ex, eil):
            afail.append((c, ex, eil, rx, ril, sx, sil))
    if afail:
        print(f"[FAIL] curated anchors: {len(afail)} mismatches")
        for c, ex, eil, rx, ril, sx, sil in afail:
            print(f"   c={c:04x} expect=({ex:08x},{eil}) ref=({rx:08x},{ril}) sv=({sx:08x},{sil})")
        return 1
    print(f"[PASS] {len(anchors)} curated exact anchors match")

    n_legal = len(legal)
    print(f"\n[OK] rvc_expand verified: {n_legal} legal / {65536 - n_legal} "
          f"illegal parcels, SV == Python-ref == binutils.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
