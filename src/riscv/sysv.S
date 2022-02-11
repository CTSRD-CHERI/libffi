/* -----------------------------------------------------------------------
   ffi.c - Copyright (c) 2015 Michael Knyszek <mknyszek@berkeley.edu>
                         2015 Andrew Waterman <waterman@cs.berkeley.edu>
                         2018 Stef O'Rear <sorear2@gmail.com>

   RISC-V Foreign Function Interface

   Permission is hereby granted, free of charge, to any person obtaining
   a copy of this software and associated documentation files (the
   ``Software''), to deal in the Software without restriction, including
   without limitation the rights to use, copy, modify, merge, publish,
   distribute, sublicense, and/or sell copies of the Software, and to
   permit persons to whom the Software is furnished to do so, subject to
   the following conditions:

   The above copyright notice and this permission notice shall be included
   in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED ``AS IS'', WITHOUT WARRANTY OF ANY KIND,
   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
   HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
   DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------- */

#define LIBFFI_ASM
#include <fficonfig.h>
#include <ffi.h>

/* Define aliases so that we can handle all ABIs uniformly */

#ifdef __CHERI_PURE_CAPABILITY__
#define PTRREG(reg) c##reg
/* Could also use .macro to make it look more like an assembly instruction */
#define PTRADDI(dst, src, inc) cincoffset PTRREG(dst), PTRREG(src), inc
#define PTRMOVE(dst, src) cmove PTRREG(dst), PTRREG(src)
#else
#define PTRREG(reg) reg
#define PTRADDI(dst, src, inc) addi dst, src, inc
#define PTRMOVE(dst, src) mv dst, src
#endif

#ifdef __CHERI_PURE_CAPABILITY__
#define PTRS __SIZEOF_POINTER__
#define LARG clc
#define SARG csc
#elif __SIZEOF_POINTER__ == 8
#define PTRS 8
#define LARG ld
#define SARG sd
#else
#define PTRS 4
#define LARG lw
#define SARG sw
#endif

#define LOAD_ARG(dst, imm, base) LARG PTRREG(dst), imm(PTRREG(base))
#define STORE_ARG(value, imm, base) SARG PTRREG(value), imm(PTRREG(base))

#if __riscv_float_abi_double
#define FLTS 8
#ifdef __CHERI_PURE_CAPABILITY__
#define FLARG cfld
#define FSARG cfsd
#else
#define FLARG fld
#define FSARG fsd
#endif
#elif __riscv_float_abi_single
#define FLTS 4
#define FLARG flw
#define FSARG fsw
#else
#define FLTS 0
#endif

#if FLTS
#define LOAD_FLOAT(dst, imm, base) FLARG dst, imm(PTRREG(base))
#define STORE_FLOAT(value, imm, base) FSARG value, imm(PTRREG(base))
#endif

#define fp s0

    .text
    .globl  ffi_call_asm
    .type   ffi_call_asm, @function
    .hidden ffi_call_asm
/*
  struct call_context {
      floatreg fa[8];
      intreg a[8];
      intreg pad[rv32 ? 2 : 0];
      intreg save_fp, save_ra;
  }
  void ffi_call_asm (size_t *stackargs, struct call_context *regargs,
                     void (*fn) (void), void *closure);
*/

#if __SIZEOF_POINTER__ < 8
#define FRAME_PADDING 8
#else
#define FRAME_PADDING 0
#endif

#define FRAME_LEN (8 * FLTS + 8 * PTRS + FRAME_PADDING + 2 * PTRS)

ffi_call_asm:
    .cfi_startproc

    /*
      We are NOT going to set up an ordinary stack frame.  In order to pass
      the stacked args to the called function, we adjust our stack pointer to
      a0, which is in the _caller's_ alloca area.  We establish our own stack
      frame at the end of the call_context.

      Anything below the arguments will be freed at this point, although we
      preserve the call_context so that it can be read back in the caller.
    */

    .cfi_def_cfa 11, FRAME_LEN # interim CFA based on a1
    STORE_ARG(fp, FRAME_LEN - 2*PTRS, a1)
    .cfi_offset 8, -2*PTRS
    STORE_ARG(ra, FRAME_LEN - 1*PTRS, a1)
    .cfi_offset 1, -1*PTRS

    PTRADDI(fp, a1, FRAME_LEN)
#ifdef __CHERI_PURE_CAPABILITY__
    /* Restore csp bounds on cfp */
    csetaddr cfp, csp, fp
#endif
    PTRMOVE(sp, a0)
    .cfi_def_cfa 8, 0 # our frame is fully set up

    # Load arguments
    PTRMOVE(t1, a2)
    PTRMOVE(t2, a3)

#if FLTS
    LOAD_FLOAT(fa0, -FRAME_LEN+0*FLTS, fp)
    LOAD_FLOAT(fa1, -FRAME_LEN+1*FLTS, fp)
    LOAD_FLOAT(fa2, -FRAME_LEN+2*FLTS, fp)
    LOAD_FLOAT(fa3, -FRAME_LEN+3*FLTS, fp)
    LOAD_FLOAT(fa4, -FRAME_LEN+4*FLTS, fp)
    LOAD_FLOAT(fa5, -FRAME_LEN+5*FLTS, fp)
    LOAD_FLOAT(fa6, -FRAME_LEN+6*FLTS, fp)
    LOAD_FLOAT(fa7, -FRAME_LEN+7*FLTS, fp)
#endif

    LOAD_ARG(a0, -FRAME_LEN+8*FLTS+0*PTRS, fp)
    LOAD_ARG(a1, -FRAME_LEN+8*FLTS+1*PTRS, fp)
    LOAD_ARG(a2, -FRAME_LEN+8*FLTS+2*PTRS, fp)
    LOAD_ARG(a3, -FRAME_LEN+8*FLTS+3*PTRS, fp)
    LOAD_ARG(a4, -FRAME_LEN+8*FLTS+4*PTRS, fp)
    LOAD_ARG(a5, -FRAME_LEN+8*FLTS+5*PTRS, fp)
    LOAD_ARG(a6, -FRAME_LEN+8*FLTS+6*PTRS, fp)
    LOAD_ARG(a7, -FRAME_LEN+8*FLTS+7*PTRS, fp)

    /* Call */
#ifdef __CHERI_PURE_CAPABILITY__
    cjalr   ct1
#else
    jalr    t1
#endif

    /* Save return values - only a0/a1 (fa0/fa1) are used */
#if FLTS
    STORE_FLOAT(fa0, -FRAME_LEN+0*FLTS, fp)
    STORE_FLOAT(fa1, -FRAME_LEN+1*FLTS, fp)
#endif

    STORE_ARG(a0, -FRAME_LEN+8*FLTS+0*PTRS, fp)
    STORE_ARG(a1, -FRAME_LEN+8*FLTS+1*PTRS, fp)

    /* Restore and return */
    PTRADDI(sp, fp, -FRAME_LEN)
    .cfi_def_cfa 2, FRAME_LEN
    LOAD_ARG(ra, -1*PTRS, fp)
    .cfi_restore 1
    LOAD_ARG(fp, -2*PTRS, fp)
    .cfi_restore 8
#ifdef __CHERI_PURE_CAPABILITY__
    cret
#else
    ret
#endif
    .cfi_endproc
    .size   ffi_call_asm, .-ffi_call_asm


/*
  ffi_closure_asm. Expects address of the passed-in ffi_closure in t1.
  void ffi_closure_inner (ffi_cif *cif,
		          void (*fun) (ffi_cif *, void *, void **, void *),
		          void *user_data,
		          size_t *stackargs, struct call_context *regargs)
*/

    .globl ffi_closure_asm
    .hidden ffi_closure_asm
    .type ffi_closure_asm, @function
ffi_closure_asm:
    .cfi_startproc

    PTRADDI(sp,  sp, -FRAME_LEN)
    .cfi_def_cfa_offset FRAME_LEN

    /* make a frame */
    STORE_ARG(fp, FRAME_LEN - 2*PTRS, sp)
    .cfi_offset 8, -2*PTRS
    STORE_ARG(ra, FRAME_LEN - 1*PTRS, sp)
    .cfi_offset 1, -1*PTRS
    PTRADDI(fp, sp, FRAME_LEN)

    /* save arguments */
#if FLTS
    STORE_FLOAT(fa0, 0*FLTS, sp)
    STORE_FLOAT(fa1, 1*FLTS, sp)
    STORE_FLOAT(fa2, 2*FLTS, sp)
    STORE_FLOAT(fa3, 3*FLTS, sp)
    STORE_FLOAT(fa4, 4*FLTS, sp)
    STORE_FLOAT(fa5, 5*FLTS, sp)
    STORE_FLOAT(fa6, 6*FLTS, sp)
    STORE_FLOAT(fa7, 7*FLTS, sp)
#endif

    STORE_ARG(a0, 8*FLTS+0*PTRS, sp)
    STORE_ARG(a1, 8*FLTS+1*PTRS, sp)
    STORE_ARG(a2, 8*FLTS+2*PTRS, sp)
    STORE_ARG(a3, 8*FLTS+3*PTRS, sp)
    STORE_ARG(a4, 8*FLTS+4*PTRS, sp)
    STORE_ARG(a5, 8*FLTS+5*PTRS, sp)
    STORE_ARG(a6, 8*FLTS+6*PTRS, sp)
    STORE_ARG(a7, 8*FLTS+7*PTRS, sp)

    /* enter C */
    LOAD_ARG(a0, FFI_TRAMPOLINE_SIZE+0*PTRS, t1)
    LOAD_ARG(a1, FFI_TRAMPOLINE_SIZE+1*PTRS, t1)
    LOAD_ARG(a2, FFI_TRAMPOLINE_SIZE+2*PTRS, t1)
    PTRADDI(a3, sp, FRAME_LEN)
    PTRMOVE(a4, sp)

#ifdef __CHERI_PURE_CAPABILITY__
    cllc    cra, ffi_closure_inner
    cjalr   cra
#else
    call    ffi_closure_inner
#endif

    /* return values */
#if FLTS
    LOAD_FLOAT(fa0, 0*FLTS, sp)
    LOAD_FLOAT(fa1, 1*FLTS, sp)
#endif

    LOAD_ARG(a0, 8*FLTS+0*PTRS, sp)
    LOAD_ARG(a1, 8*FLTS+1*PTRS, sp)

    /* restore and return */
    LOAD_ARG(ra, FRAME_LEN-1*PTRS, sp)
    .cfi_restore 1
    LOAD_ARG(fp, FRAME_LEN-2*PTRS, sp)
    .cfi_restore 8
    PTRADDI(sp, sp, FRAME_LEN)
    .cfi_def_cfa_offset 0
#ifdef __CHERI_PURE_CAPABILITY__
    cret
#else
    ret
#endif
    .cfi_endproc
    .size ffi_closure_asm, .-ffi_closure_asm

/*
  ffi_go_closure_asm.  Expects address of the passed-in ffi_go_closure in t2.
  void ffi_closure_inner (ffi_cif *cif,
		          void (*fun) (ffi_cif *, void *, void **, void *),
		          void *user_data,
		          size_t *stackargs, struct call_context *regargs)
*/

    .globl ffi_go_closure_asm
    .hidden ffi_go_closure_asm
    .type ffi_go_closure_asm, @function
ffi_go_closure_asm:
    .cfi_startproc

    PTRADDI(sp,  sp, -FRAME_LEN)
    .cfi_def_cfa_offset FRAME_LEN

    /* make a frame */
    STORE_ARG(fp, FRAME_LEN - 2*PTRS, sp)
    .cfi_offset 8, -2*PTRS
    STORE_ARG(ra, FRAME_LEN - 1*PTRS, sp)
    .cfi_offset 1, -1*PTRS
    PTRADDI(fp, sp, FRAME_LEN)

    /* save arguments */
#if FLTS
    STORE_FLOAT(fa0, 0*FLTS, sp)
    STORE_FLOAT(fa1, 1*FLTS, sp)
    STORE_FLOAT(fa2, 2*FLTS, sp)
    STORE_FLOAT(fa3, 3*FLTS, sp)
    STORE_FLOAT(fa4, 4*FLTS, sp)
    STORE_FLOAT(fa5, 5*FLTS, sp)
    STORE_FLOAT(fa6, 6*FLTS, sp)
    STORE_FLOAT(fa7, 7*FLTS, sp)
#endif

    STORE_ARG(a0, 8*FLTS+0*PTRS, sp)
    STORE_ARG(a1, 8*FLTS+1*PTRS, sp)
    STORE_ARG(a2, 8*FLTS+2*PTRS, sp)
    STORE_ARG(a3, 8*FLTS+3*PTRS, sp)
    STORE_ARG(a4, 8*FLTS+4*PTRS, sp)
    STORE_ARG(a5, 8*FLTS+5*PTRS, sp)
    STORE_ARG(a6, 8*FLTS+6*PTRS, sp)
    STORE_ARG(a7, 8*FLTS+7*PTRS, sp)

    /* enter C */
    LOAD_ARG(a0, 1*PTRS, t2)
    LOAD_ARG(a1, 2*PTRS, t2)
    PTRMOVE(a2, t2)
    PTRADDI(a3, sp, FRAME_LEN)
    PTRMOVE(a4, sp)

#ifdef __CHERI_PURE_CAPABILITY__
    cllc    cra, ffi_closure_inner
    cjalr   cra
#else
    call    ffi_closure_inner
#endif

    /* return values */
#if FLTS
    LOAD_FLOAT(fa0, 0*FLTS, sp)
    LOAD_FLOAT(fa1, 1*FLTS, sp)
#endif

    LOAD_ARG(a0, 8*FLTS+0*PTRS, sp)
    LOAD_ARG(a1, 8*FLTS+1*PTRS, sp)

    /* restore and return */
    LOAD_ARG(ra, FRAME_LEN-1*PTRS, sp)
    .cfi_restore 1
    LOAD_ARG(fp, FRAME_LEN-2*PTRS, sp)
    .cfi_restore 8
    PTRADDI(sp, sp, FRAME_LEN)
    .cfi_def_cfa_offset 0
#ifdef __CHERI_PURE_CAPABILITY__
    cret
#else
    ret
#endif
    .cfi_endproc
    .size ffi_go_closure_asm, .-ffi_go_closure_asm
