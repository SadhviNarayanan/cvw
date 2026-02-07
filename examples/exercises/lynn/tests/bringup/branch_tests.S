// Branch instruction tests
// Tests: bne, blt, bltu, bge, bgeu

.extern return_branch_tests
.extern fail

    .section .text.init
    .globl branch_tests
    .type  branch_tests, @function

branch_tests:

    // BNE tests
    test_bne_1:
        li   t0, 5
        li   t1, 10
        bne  t0, t1, test_bne_2
        j    Fail_Bne_1

    test_bne_2:
        li   t0, 5
        li   t1, 5
        bne  t0, t1, Fail_Bne_2
        j    test_blt_1

    // BLT tests
    test_blt_1:
        li   t0, 5
        li   t1, 10
        blt  t0, t1, test_blt_2
        j    Fail_Blt_1

    test_blt_2:
        li   t0, -5
        li   t1, 10
        blt  t0, t1, test_blt_3
        j    Fail_Blt_2

    test_blt_3:
        li   t0, 10
        li   t1, 5
        blt  t0, t1, Fail_Blt_3
        j    test_bltu_1

    // BLTU tests
    test_bltu_1:
        li   t0, 5
        li   t1, 10
        bltu t0, t1, test_bltu_2
        j    Fail_Bltu_1

    test_bltu_2:
        li   t0, 5
        li   t1, 0xFFFFFFFF
        bltu t0, t1, test_bltu_3
        j    Fail_Bltu_2

    test_bltu_3:
        li   t0, 0xFFFFFFFF
        li   t1, 5
        bltu t0, t1, Fail_Bltu_3
        j    test_bge_1

    // BGE tests
    test_bge_1:
        li   t0, 10
        li   t1, 5
        bge  t0, t1, test_bge_2
        j    Fail_Bge_1

    test_bge_2:
        li   t0, 5
        li   t1, 5
        bge  t0, t1, test_bge_3
        j    Fail_Bge_2

    test_bge_3:
        li   t0, 5
        li   t1, 10
        bge  t0, t1, Fail_Bge_3
        j    test_bgeu_1

    // BGEU tests
    test_bgeu_1:
        li   t0, 10
        li   t1, 5
        bgeu t0, t1, test_bgeu_2
        j    Fail_Bgeu_1

    test_bgeu_2:
        li   t0, 0xFFFFFFFF
        li   t1, 5
        bgeu t0, t1, test_bgeu_3
        j    Fail_Bgeu_2

    test_bgeu_3:
        li   t0, 5
        li   t1, 10
        bgeu t0, t1, Fail_Bgeu_3
        j    pass

    pass:
        j return_branch_tests

Fail_Bne_1:
    li   gp, 439
    j    fail

Fail_Bne_2:
    li   gp, 440
    j    fail

Fail_Blt_1:
    li   gp, 441
    j    fail

Fail_Blt_2:
    li   gp, 442
    j    fail

Fail_Blt_3:
    li   gp, 443
    j    fail

Fail_Bltu_1:
    li   gp, 444
    j    fail

Fail_Bltu_2:
    li   gp, 445
    j    fail

Fail_Bltu_3:
    li   gp, 446
    j    fail

Fail_Bge_1:
    li   gp, 447
    j    fail

Fail_Bge_2:
    li   gp, 448
    j    fail

Fail_Bge_3:
    li   gp, 449
    j    fail

Fail_Bgeu_1:
    li   gp, 450
    j    fail

Fail_Bgeu_2:
    li   gp, 451
    j    fail

Fail_Bgeu_3:
    li   gp, 452
    j    fail
