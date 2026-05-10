[33mcommit f909db29a4230c496647957aba49af7669af7300[m[33m ([m[1;36mHEAD[m[33m -> [m[1;32mTestBranch_3[m[33m, [m[1;31morigin/TestBranch_3[m[33m)[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu Apr 30 21:36:25 2026 -0500

    Researched and optimized registers a bit, should not fry boards (hopefully)

[33mcommit d8f4d071456e6d93f2e2975cd9a013646bc4d76e[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun Mar 29 20:42:48 2026 -0500

    Good Test Commit, almost balances, good motor torque, needs a little more smoothness

[33mcommit 208f5c915263070df9cceec516ec016e8400fe20[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun Mar 29 20:26:01 2026 -0500

    Made definitions for tunable parameters, started configuring the gyro, verified TDC with pendulum Hall Effect sensors

[33mcommit 772fd42aece0573c9e0a6a4e667cb65fa438f281[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Mar 28 22:28:47 2026 -0500

    Best operation yet

[33mcommit c80b9b11ee5cc7ca7b6240c68c812de933486ceb[m[33m ([m[1;32mmain[m[33m)[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Mar 28 20:33:00 2026 -0500

    Used set_param_chip1 functions for chip2 (2nd IHM02A1), because the fron motor wasnt getting configured

[33mcommit 0ca4cf4960e987254bc137bdd3497f13bc53d981[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Mar 28 20:09:50 2026 -0500

    More tuning and adjustments. We are almost there. But noticed the front wheel does not have the power of the other two. Also need to handle rotation and further tuning.

[33mcommit 87295f57b8dabb0212ddf4b2f5b20662e9909a71[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Mar 28 13:41:27 2026 -0500

    Implemented Busy check for motors before sending omni_drive() command, still need to test and verify functionality

[33mcommit 4cfb307ffecd2aec7e4096d1a6a5588b80ab8398[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed Mar 25 20:16:18 2026 -0500

    Cleaned TODOS

[33mcommit 368f03bda33f51af5ff061a38e0278c0db988d95[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Mar 24 22:15:21 2026 -0500

    Changed the SPI baudrates to 16 for 2.5 MHz SPI CLK (MAX 5 GHz), also made all SPI transmit function use HAL_MAX_DELAY

[33mcommit 4a78e04e98ba35afdb8ea9d1b57934484f5d9270[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Mar 24 22:04:01 2026 -0500

    Removed a todo comment

[33mcommit 9de94171c415c4fcc895fc67fe843c13203c0a71[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Mar 24 22:03:35 2026 -0500

    Fixed some warnings in spd_tx_buffer size

[33mcommit b65e2d09453b690c5b5b74a9d09a837dd52ed854[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Mar 24 21:55:22 2026 -0500

    Optimized code a little bit, included optimization for speed as project setting, found 6 bugs that need to be fixed

[33mcommit c396f0f652451cc61490f9575466e1685bf0e072[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Mar 24 21:02:36 2026 -0500

    Cleaned some code, fixed Y angle by multiplying by -1, configured control system so it attempts to balance

[33mcommit 42fd79b5c5facdcd32c7e4a8b4bf70d5465e760c[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Mar 24 20:33:20 2026 -0500

    Added MATLAB parameters in comments

[33mcommit dcfdaf972cc9e151413cf2942395c47ded04472d[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Mar 24 20:25:23 2026 -0500

    Properly normalized Hall Effect voltages in mapVoltageToAngle, organize IMU code into two new files

[33mcommit e87953a609a89c2f4381c6b01574cbacab1f8fcf[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Mar 24 20:03:05 2026 -0500

    Updated Push Button Config to Pull Down an Rising Edge INT, added some debugging statements

[33mcommit e6e49a354cf161ead5937c5421ba610b6176fef7[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Mar 21 12:55:21 2026 -0500

    Added a comment about clock prescalers

[33mcommit 0fd2d3fdb9ee0dd38689077b6b8ccd7be4e5c7f4[m[33m ([m[1;31morigin/main[m[33m, [m[1;31morigin/HEAD[m[33m)[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Feb 17 19:14:21 2026 -0600

    Fixed the clock speeds for both SPI1 and SPI2, had to fix some user code sections that got duplicated somehow. Still need to test. If it doesnt work, go back 1 commit

[33mcommit de3ffe42446a0a8b5b9798444ad1b541ed9b5bad[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu Jan 1 17:48:47 2026 -0600

    Prepared the codebase to verify the POT voltages and angles

[33mcommit c2eaf7c78cd1c7614005f25ee5c9c313ddf5c373[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu Jan 1 16:45:12 2026 -0600

    Removed Unused/Old ThesisProject code

[33mcommit ed881142453b152cbad821716509b60e395186f6[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Dec 2 20:23:46 2025 -0600

    Successfully reading IMU values in main while loop

[33mcommit 16cad2c95b3fc6e92f44275198cef73827e5f666[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Dec 2 19:21:06 2025 -0600

    Finished IMU config function

[33mcommit 4e4526f7ff185ba08d56a671e41cffdc11a98852[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Dec 1 20:46:11 2025 -0600

    Started IMU Config function

[33mcommit 0fd0df0f5c92ed021845337417abbc5a0cf96df2[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Fri Nov 21 22:18:21 2025 -0600

    Reading WhoAmI register on MPU6000

[33mcommit 0c1126910606e5cbd65b1fa265b6fab658e57619[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun Nov 16 19:13:31 2025 -0600

    Implemented low pass IIF filter and D control paramter

[33mcommit 376f02b491e0048194521074ca0ad5f50e07d87a[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Nov 15 19:55:45 2025 -0600

    First iteration of control system

[33mcommit 5e43d39bffe867a34f852b8d65befd031d1d9773[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Nov 15 12:48:44 2025 -0600

    defined MAX_SPEED_RAD as 10 PI and convert degrees to radians in mapVoltageToAngle()

[33mcommit b482a4559a8651faef0b0e41c5e40ce2926ab5bb[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Fri Nov 14 20:44:46 2025 -0600

    Optimized, took out delays, cleaned indentation and comments

[33mcommit 84e30afd07f2e9c9db416129ad2cdfe9a8cb86d5[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Oct 18 12:57:44 2025 -0500

    Got robot to move in the correct direction

[33mcommit 7759281eab04af8cd69379744f23f1c7bf10e5fa[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Oct 18 00:45:18 2025 -0500

    Tracking goes the opposite direction. Need to normalize speed values

[33mcommit eb1ad8957521b748a19fd67919f74ccb10304bb6[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Oct 18 00:15:34 2025 -0500

    Working balancing act, but goes opposite direction it should

[33mcommit 81ef7813040e905bb5654e80a6fed2f39b2f6936[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu Oct 16 21:00:23 2025 -0500

    Made the wheels and pendulum have the same coordinate reference for x and y axes

[33mcommit d66b4a80837b8944a1afbe873109c8eb489c94ec[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu Oct 16 20:50:55 2025 -0500

    Fixed GPIO pushbutton debounce issue

[33mcommit 6b6a29ad68ff9c59bd4a495af2e03327afb3c81d[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu Oct 16 20:42:30 2025 -0500

    Updated cleaned code. Verified motor control. Verified POT data. Verified LED and pushbutton

[33mcommit 1178b98a9010307fe143edd47c09c69109992fd8[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed Oct 15 18:25:24 2025 -0500

    Updating commit

[33mcommit 59bc2d946782a53a68d9c9417e0b6bd6f54a03e5[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Oct 4 20:45:05 2025 -0500

    Re-organized the functions in the files

[33mcommit 1ac3a59f02014ecea1fb82038bea53efbe98ef3d[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Sep 20 10:38:02 2025 -0500

    enabled IMU I2C in software

[33mcommit 294bcc4a322a7d37c0a4df4fede632ade5e14778[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Sep 15 20:51:49 2025 -0500

    Configured external user push button

[33mcommit 00fdf8ccb1995560942c3b61e202270b8ba10529[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Sep 15 19:42:07 2025 -0500

    Configured LED

[33mcommit c3fa0c3284e9784bd651091b318421d5c80a4cf7[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Aug 18 19:48:31 2025 -0500

    Got second ADC to work!

[33mcommit 5484c0b9c7ca55c2850caeeabfa0c14ebcd48abd[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Aug 16 17:38:04 2025 -0500

    Added all regs to prints

[33mcommit 8cfba95b677fd7cdb5cbde79e1b0d8d42f10c536[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Aug 16 17:27:21 2025 -0500

    Fixed get params for chip 2

[33mcommit b6e72635931ae4e85d9f8e1f87d911df9c2a8736[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Aug 16 16:52:16 2025 -0500

    Removed excess testing code

[33mcommit 8ec50647a06232fdbdef343d88cde8072317bfa9[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Aug 16 16:21:50 2025 -0500

    Setup UART to work with FTDI cable

[33mcommit 247c11b8b9c2fa89f4e5002e178a218b6dcd09b7[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun Aug 10 17:13:54 2025 -0500

    Added 2nd ADC and GPIO for LED and BUTTON

[33mcommit c81b3eabf7182d0910d591e859633a643258fb54[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jun 28 12:13:11 2025 -0500

    All three motors work and the button interrupt works

[33mcommit 6df7927d3d2a05d26017bce87652abd9e156d544[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Jun 24 19:58:27 2025 -0500

    Testing Accel Function

[33mcommit b94f8ba6a1135ea25c71d80f9185c1c2ee9ee143[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Jun 24 19:32:19 2025 -0500

    Added Accel function

[33mcommit d612fe4fea657b86cdf89dc2fdfecc5405077c65[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jun 21 16:13:38 2025 -0500

    re-added .project and .cproject files

[33mcommit c423e049e29ccda16cab98db112950f30bce2b53[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jun 21 16:12:10 2025 -0500

    Started the accel function

[33mcommit 06a23aac05d6ab8f5c44856e129bd02709baaed2[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Jun 16 19:47:40 2025 -0500

    Intermediate commit while making accel function

[33mcommit a38395fa0b2cde98dcf6ab0f8dc3bdbbaba5eb0f[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Jun 16 19:16:43 2025 -0500

    Optimized KVALs

[33mcommit 0ffd8e3daf8f780658dc82807ec6a52464c268a0[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jun 14 19:39:32 2025 -0500

    Solved red light error, it was the KVAL values

[33mcommit 61f0f50f16fd0706df46c1451026c4dca2f7fd4a[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jun 14 18:52:50 2025 -0500

    Intermediate testing commit, should move forward despite red light and implement acceleration functions

[33mcommit ea464c6c75c1b64c419d26c43c27b84e9f8b779d[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jun 14 17:32:32 2025 -0500

    In progress of debugging red error light. Modified Enable, Disable, SoftStop, also added rough acceleration function

[33mcommit 8d9411312772dbe014a81018a2fcd6912a1288a4[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jun 7 17:41:37 2025 -0500

    added delay after transmit SPI

[33mcommit 7339a4d2288fd2d37013d4dbf6ca84ddb2cffa44[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jun 7 15:19:03 2025 -0500

    Troubleshooting motor control at high speeds

[33mcommit 0761076e7b80719a9da0fd1ada24cff835463ca8[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Jun 2 21:45:09 2025 -0500

    Test ready for Dr Winstead

[33mcommit fee217f1c664c9eed529b50709f48c1f81079a76[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Jun 2 21:26:46 2025 -0500

    removed another print

[33mcommit 2fd825282ed54382f39c686f2541a92bfbcda207[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Jun 2 21:25:32 2025 -0500

    Removed print statements

[33mcommit 2051076dd09566f7a4c875d8474852ba389d0628[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Jun 2 21:23:41 2025 -0500

    added math.h and converted to 1 rps

[33mcommit 81a7302527b0b31003862877c8d9634906cf90a7[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Jun 2 21:20:43 2025 -0500

    Fixed motor 1 control

[33mcommit 4b065816e5b4191e8d62c6e8aec45049b0b59bc9[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed May 28 19:59:25 2025 -0500

    Read and write register functions for chip1 and chip2

[33mcommit e0845e2d81450ef6d4edd8fd61d57fd885a0b2c0[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun May 25 19:33:47 2025 -0500

    Updated todos

[33mcommit 9ca0f9d6a7c5b316084ea443f7df6578a3dfd187[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun May 25 19:27:46 2025 -0500

    Update body radius

[33mcommit 4f50daf7b8b86bca0f5f3b72042f4a44d5e15840[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun May 25 19:26:08 2025 -0500

    Updated wheel radius

[33mcommit 59d94e4ac54b63ec9b83ecb67405bb9f98bd53ec[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun May 25 19:20:07 2025 -0500

    Optimized speed and removed most prints

[33mcommit 53a34dd4c1ebf31a43c876a3a8166dd6fcf3b980[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu May 15 21:33:48 2025 -0500

    Fixed wheel orientation

[33mcommit 50fbf745c887f1e7c8d9ea0e03b119c9f28b8711[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu May 15 20:07:09 2025 -0500

    Got all three motors to work

[33mcommit bf8ea50875a8a47b4fd542e164627ae8445ffdbb[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu May 15 19:18:14 2025 -0500

    Added soft_stop function

[33mcommit 7e62fc73b8ddf39cda8c8f44cdac155b6bb4d765[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu May 15 19:03:14 2025 -0500

    Fixed the custom spi motor run function

[33mcommit fe51f44bc14818ea80994d57ee35ba9bb38ff859[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed May 14 06:55:27 2025 -0500

    In process of modifying the spi set_vel w/o DMA, then with DMA

[33mcommit 7ad9353fd607ffb5f071780ff701e74b39690d49[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat May 3 20:31:08 2025 -0500

    Completed successful register read and writes. Troubleshooting motor 1, troubleshooting non-stopping motion

[33mcommit 1bb674217d65d36a531149cae6d86bb278f5e0d8[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat May 3 19:35:46 2025 -0500

    Register settings are correct. Now Testing motors

[33mcommit 9e8916e43975b6828e3a3cc87a0efc04615ff780[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat May 3 18:16:51 2025 -0500

    Reconfigured SPI bus CPOL and CPHA correctly

[33mcommit c99fa1aff1a24f341f1ce076fb67d123f44b850a[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat May 3 14:36:58 2025 -0500

    Finished 1-3 byte register read and write functions. 1 byte success, still testing 2-3 bytes

[33mcommit 0003aac765e28fb00684497db54d2be3183f2660[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat May 3 13:16:15 2025 -0500

    Success with one byte read and writes

[33mcommit 26d7de3a1f26c10ce60ae38dda0a06c0cbb87038[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat May 3 12:55:30 2025 -0500

    Testing register read and writes, read 1 byte register success

[33mcommit 7afb78d22cd2ea3499c76537d29efeae368eb7f1[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Apr 26 21:43:21 2025 -0500

    Success with 1 byte get param register

[33mcommit 995a5aa083b75558602cc8bf0fc452086d9adc74[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Apr 26 19:15:00 2025 -0500

    Intermediate commit while trying to configure get and set register functions

[33mcommit a4c65c36b9655010ae92d2a45d722441a6717e48[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed Apr 2 19:10:59 2025 -0500

    Fixed the HAl Delay infinite loop bug

[33mcommit 7a3d8ab44861fbaafb95bd7f1163ecd307fdab50[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Apr 1 20:09:09 2025 -0500

    Added J and J_INV and F, B, L, R functions

[33mcommit a7291986caa52f9629677480dc2d18eab4d7b572[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Mar 31 19:00:20 2025 -0500

    Added user push button and removed some print statements

[33mcommit bb2115657f23a937defc8674827bff981c04cde2[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Mon Mar 24 20:13:39 2025 -0500

    Individual motors work, but batteries were dead

[33mcommit 4427667613dafd4faa5c01f5671173bbcfb5f7ea[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun Mar 23 18:16:05 2025 -0500

    fixed SPI TX errors

[33mcommit bdd988b7d549bf228e3afc378b734dd3a3da4879[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun Mar 23 17:04:49 2025 -0500

    added stepper_motor->num_motors

[33mcommit 7112a20e9cefea08d03d7fa9fc60684efe2b2a71[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun Mar 23 16:46:04 2025 -0500

    Added DMA for SPI2

[33mcommit e89c720aef479eec042123161a25569703d2022e[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun Mar 23 16:35:17 2025 -0500

    Added comments for the set_vel function

[33mcommit 17b63e28b3af723944acc2018e336f62bb8ea64e[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed Mar 19 22:14:04 2025 -0500

    Working printf statements

[33mcommit 096a51c56db055f7a4e9f5a5dbd1f5133881dffd[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed Mar 19 11:02:03 2025 -0500

    Removed .metadata files from Git tracking

[33mcommit bc050a4c8cec3c8ad85074211447c278071ea844[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Mar 15 19:31:05 2025 -0500

    Updated SPI2 Clock Prescaler

[33mcommit 0e08008f24b3fb0ca797e53fc0aaa450f34b8c4f[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu Mar 6 18:48:47 2025 -0600

    Added configuration for the 2nd SPI for top shield, tested, but still needs troubleshooting

[33mcommit 8b8c6ca8971d6b43bcb2021c9346d7073787e748[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Thu Mar 6 16:55:03 2025 -0600

    Updated troubleshooting code for asynchronous motor operation

[33mcommit 14051de57c29526f04cfa8d24e3059bb602d66cf[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed Mar 5 15:37:22 2025 -0600

    Progress

[33mcommit ac014476cad2b5267bd68ab940f398419f6db44a[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed Mar 5 15:27:12 2025 -0600

    Initial Commit with last working Code

[33mcommit b245e774249e9c1e717a73697084e3b48fc1352e[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Feb 25 19:41:13 2025 -0600

    Attempt at moving the steppers individually, but unsuccessful so far

[33mcommit b33466099108069a9f0bf4ba54d1732df926a704[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Feb 15 19:56:10 2025 -0600

    2 direction motion

[33mcommit ae7e03d99f527916943c071b1c3b0b0f6ec81a4e[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Feb 15 19:35:02 2025 -0600

    Added some more configurations

[33mcommit dd006031873a0eb5d2d814c41782c9165afa25c9[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Feb 15 17:49:40 2025 -0600

    Added working L6470 motor control codebase

[33mcommit 180128ac05e6269c4359150c2813859e445a2b21[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Feb 1 18:17:21 2025 -0600

    Added debugging statements

[33mcommit 0410d0d6be7384bb7e75d6da90bd99a7e3367898[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Feb 1 17:55:30 2025 -0600

    Modified .gitignore

[33mcommit ba15aa3f90c8e54be3bfbc5fe668c1cd67c534b8[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Feb 1 15:37:21 2025 -0600

    Compiled with no errors commit, but untested

[33mcommit e127d90734b661650c6c1b21a7e5261ac031ca35[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Jan 28 20:18:44 2025 -0600

    Started working on l6474.c file

[33mcommit 89de24bb6a8354f8bad2c5f03852532c7e072c7d[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Jan 28 19:42:30 2025 -0600

    l6474 header progress commit

[33mcommit b90662bcdec3726e14e29dbca47c39e5b689f7e7[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Jan 28 19:15:36 2025 -0600

    Good Starting Point Commit

[33mcommit dbfa1bed02b9bc93760ab10821b21ca2c74b97a8[m[33m ([m[1;31morigin/AttemptToFixErrorsBranch[m[33m)[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Tue Jan 28 18:56:02 2025 -0600

    Deleted Old Projects

[33mcommit 627fceeb1a17217f7e1c413cf1cd1de8d023cd88[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jan 25 14:20:44 2025 -0600

    Tried, but unsuccessful with F410

[33mcommit 120f0158738ff227e8d5f75eb6a827b666d88e6b[m
Merge: b99c62f 38cbf77
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Fri Jan 24 20:32:20 2025 -0600

    Merge branch 'AttemptToFixErrorsBranch' of https://github.com/CodeBlue-117/Thesis into AttemptToFixErrorsBranch

[33mcommit b99c62f5f6051885558aa0e141bbb0f807de25aa[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Fri Jan 24 20:31:52 2025 -0600

    Found GPIOS to initialize

[33mcommit 38cbf771654a872ea477432c13167c55eabba21d[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Fri Jan 24 20:24:31 2025 -0600

    Found GPIO pins that need to be defined

[33mcommit 459ebd881b7f4048a0483c15a59d16a1cdb64467[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Fri Jan 24 20:01:27 2025 -0600

    Ported over all source code, but need to fix interrupt handlers in startup file

[33mcommit 1349ac4589c5792946033128c4f51d8aff8b2a36[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed Jan 22 19:26:39 2025 -0600

    In-Progress but broken

[33mcommit b4c1005ff6ad12111336fb8c7e321829e003e303[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Wed Jan 22 19:07:16 2025 -0600

    Start to fix the errors commit

[33mcommit deb7912a08603a1e2523d3102afa027e049e356f[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sun Jan 19 17:40:40 2025 -0600

    Added ADC files

[33mcommit eae418a6a7b6884e156fa37ae76b27e4c3223bff[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jan 18 20:58:41 2025 -0600

    Working Commit

[33mcommit 8ff6d697f570ae311180118cd742c6281e798b4f[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jan 18 20:02:17 2025 -0600

    Initial Fixed Commit

[33mcommit 6fe6f0f4e113058ce749da7eb7c06bd20118aa2d[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jan 18 19:46:34 2025 -0600

    Added .gitignore

[33mcommit c0452ebbe279dca4ce8ea6e2d35fd3d1a7a2221d[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jan 18 19:44:35 2025 -0600

    Added STM32 Stepper Project in its own directory

[33mcommit 196ea5df9d78ed27105de3d5ebb005846f02324c[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Sat Jan 18 19:38:42 2025 -0600

    Add STM32 Stepper Motor Project

[33mcommit e6c8d1b51be2f63ebe6ecf19881d987af2c986e6[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Fri Jun 21 20:40:46 2024 -0500

    Update Commit

[33mcommit f65e9cae2d149c07d36c66f49d952d9efbfeaa9b[m
Author: CodeBlue-117 <jacob.price@nextgenrf.com>
Date:   Fri Jun 21 19:47:47 2024 -0500

    My First Practice Model

[33mcommit 70e804f5f98900135ad0a814253367394995e6da[m
Author: Jake Price <133265278+CodeBlue-117@users.noreply.github.com>
Date:   Fri Jun 21 19:41:28 2024 -0500

    Initial commit
