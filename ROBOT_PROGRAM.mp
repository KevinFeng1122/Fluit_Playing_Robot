NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   008C   0008 =      8. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       008C  __lit_start
       0094  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           idata   0094   094C =   2380. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0094  __idata_start
       09E0  __idata_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   09E0   0910 =   2320. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       04F0  __start
       04F0  __text_start
       0512  _exit
       0513  _delay
       0528  _I0_init
       052D  _uart0_init
       055B  _UART0_Receive_Byte
       056B  _uart1_init
       05A0  _UART1_SendB
       05B0  _UART1_Send_String
       05C8  _Play_Music
       0633  _Play_Note
       06BD  _Numeric_Valve
       06C3  _Direction_Valve
       06DA  _main
       071F  push_arg4
       0721  push_arg2
       0724  div32u
       0726  mod32u
       072A  div32s
       072C  mod32s
       0758  long_div_prolog
       076B  long_div_epilog
       077B  tst_R16_R19
       0780  tst_R24_R27
       0785  neg_R24_R27
       078E  elpm32
       07A2  pop_xgset003C
       07A7  pop_xgset00FC
       07AE  push_xgsetF0FC
       07B0  push_xgset30FC
       07B2  push_xgset00FC
       07B4  push_xgset003C
       07B9  push_xgsetF000
       07BE  pop_xgsetF000
       07C3  push_xgsetF00C
       07CA  pop_xgsetF00C
       07D1  neg32
       07DA  fpsub
       07DC  fpadd
       07DD  fpadd_alt
       0844  fpint
       0876  unpacks
       087E  tstR16_R19
       0886  unpacku
       089F  normalize_and_pack
       08D4  pack
       08E4  saveFPRegs
       08F3  restoreFPRegs
       0902  fpmule
       0955  fpmule1
       095F  fpcmp
       096C  fpcmp2
       0978  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0100   094C =   2380. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  __data_start
       0100  _uart0_received_flag
       0101  _Note_end
       0102  _Note_test1
       010F  _Note_test2
       011D  _Note_test3
       012B  _Note_test4
       0139  _KangDingQingGe
       0459  _YueGuangXiaDeFengWeiZhu
       0779  _N25
       07C2  _N26
       080B  _N27
       0853  _N1
       089B  _N2
       08E2  _N3
       0929  _N4
       0972  _N5
       09B8  _N6
       09FE  _All_Rise
       0A4C  __data_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0A4C   0001 =      1. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0A4C  _uart0_received_byte
       0A4C  __bss_start
       0A4D  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   004C =     76. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

C:\iccv7avr\lib\crtatmega.o	[ crtatmega.s ]
Robot_Program.o	[ Robot_Program.c ]
<library>	[ asave.s, div32.s, elpm32.s, gpop2.s, gpop3.s, gpush.s, gpushy.s, gpushz.s, lneg.s, fpadd.s, fpint.s, fputil.s ]
[ fpmule.s, fpmule1.s, fpcmp.s, fpcmp2.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x1e

User Base Address Definitions

func_lit = 0x8c
data = 0x100
eeprom:0.4096

