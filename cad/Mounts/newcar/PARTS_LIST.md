# New car: parts list

Prices in USD as read on 2026-09-05 from the linked page unless marked "approx." (Amazon
listings could not be fetched; those are typical listing prices and should be checked at
order time). The power tree these parts implement is NEWCAR_LAYOUT.md section 6.

## A. Already in hand (not costed)

| item | what it is | spec source |
| --- | --- | --- |
| chassis "KA2246-R00" | Traxxas Slash 4x4, standard 6822 chassis (68086-4 / 68054 / 68154-4 family), with bulkhead 6830, servo saver set 6845X, motor in the chassis | hobbytown.com/traxxas-slash-4x4-vxl-brushless-1-10-4wd-rtr-short-course-truck-fox-tra68086-4-fox/p630449; rcscrapyard.net/manuals/traxxas/68086-4/68086-4-002.jpg |
| Jetson Orin Nano Developer Kit | carrier P3768, DC jack 5.5 x 2.5 mm, 9-20 V, 3.5 A | docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html |
| brushless motor | 36 mm class assumed for the keep-out | - |
| INJORA INJS235 + 25T horn | 35 kg.cm brushless servo, 4.8-8.4 V | injora.com/products/injora-injs235-35kg-waterproof-brushless-servo-large-torque-digital-servo |
| Orbbec Gemini 335 | depth camera, USB-C | orbbec.com |
| SICK TiM561-2050101 (1071419) | 270 degree lidar, 9-28 V, M12 5-pin power, M12 D-coded Ethernet | sick.com/media/pdf/6/46/446/dataSheet_TiM561-2050101_1071419_en.pdf |
| donor M12 power cable | must be 5-pin A-coded female, brown = +V (pin 1), blue = 0 V (pin 3) | SICK TI 8015883 p.14 |
| donor M12-to-RJ45 cable | must be 4-pin D-coded male to RJ45 | SICK OI 8015886 |
| 4S LiPo with XT90 | the old car's SMC 9000 type; if a second pack is needed see C |  |
| old-set prints | `../out/jetson_orin_nano_under.stl`, `../out/camera_mast_head.stl`, `../out/cable_tidy_edge.stl`, `../out/vesc_tub_bracket.stl`, `../out/clipless_foot.stl` (x 14), and 12 clipless pieces from `../templates/Clipless_Mounting_Piece.stl` | README.md |

## B. To buy

| # | item | qty | unit | source | note |
| --- | --- | --- | --- | --- | --- |
| 1 | Flipsky FSESC 6.7 PRO, aluminium case with anti-spark switch (VC-67PS-S-060) | 1 | 107.00 | https://flipsky.net/products/flipsky-fsesc-6-7-pro-based-upon-vesc6-with-aluminum-case-and-anti-spark-switch-with-heat-sink | 8-60 V, 60 A cont.; parity with the old car; the 5 V aux rail is 1.5 A, hence #4 |
| 2 | Baseus Blade HD 100 W, 20,000 mAh (PPBL000301) | 1 | 69.99 | https://www.baseus.com/products/blade-laptop-power-bank-100w-20000mah (out of stock there; https://www.walmart.com/ip/5367455181) | 133.9 x 133.9 x 17.8, 445 g; confirm the 15 V 3 A PDO; the 161.8 mm Blade 100 W (129.99) does not fit the tray on the car |
| 3 | Adafruit 5451, USB-C PD to 5.5 x 2.5 mm barrel, 15 V 5 A, 1.2 m | 1 | 7.95 | https://www.adafruit.com/product/5451 (out of stock at Adafruit: https://www.digikey.com/en/products?mpart=5451&v=1528) | 5449 in the brief is the 9 V one of the same family; 5.5 x 2.5 centre-positive, fits 2.1 too |
| 4 | Hobbywing UBEC 5A Air, 2-8S, 5.0/6.0/7.4 V | 1 | 13.99 | https://www.hobbywingdirect.com/products/ubec-5a-air | servo supply at 7.4 V, 5 A cont., 15 A peak |
| 5 | XT90 pigtails, 10 AWG, 2 pairs (Amass) | 1 pack | 12 approx. | https://www.amazon.com/Pigtails-XT-90S-Connector-Silicon-Battery/dp/B07TBDTM2D | LiPo Y: 1 female + 2 male |
| 6 | XT60 pigtails, 12 AWG, pairs | 1 pack | 12 approx. | https://www.amazon.com/10AWG-Female-Connector-Silicon-Silicone/dp/B0DKSL2JVK | BEC tap |
| 7 | inline ATO/ATC blade fuse holders, 12 AWG, 5-pack | 1 | 10 approx. | https://www.amazon.com/MCIGICM-Inline-Fuse-Holder-Blade/dp/B081DHT8Y7 | two used: lidar rail, BEC input |
| 8 | blade fuse assortment 1-40 A | 1 | 10 approx. | https://www.amazon.com/130PCS-Standard-Assortment-SIM-NAT/dp/B07G33XCHM | 1 A (lidar; SICK says 0.8 A slow-blow, Littelfuse 0287001 if a 1 A ATO is not in the kit) and 5 A (BEC) |
| 9 | silicone wire 10 AWG red + black, 3 m each | 1 | 15 approx. | Amazon "10 AWG silicone wire" | LiPo to VESC |
| 10 | silicone wire kit 18 / 22 AWG | 1 | 15 approx. | Amazon | BEC, lidar and barrel leads |
| 11 | M3 heat-set inserts, 4.0 OD x 5 mm, brass, 100 | 1 | 9 approx. | https://www.amazon.com/Threaded-Inserts-Printing-Embedment-Automotive/dp/B0FDKYTMZ8 | VESC tray bosses (optional) |
| 12 | M2.5 heat-set inserts, 3.5 OD, kit | 1 | 12 approx. | https://www.amazon.com/Kadrick-Threaded-Insert-Kit/dp/B0FR8DP3T6 | Jetson tray bosses (optional screws) |
| 13 | M3 socket-head screw assortment (needs 4 x M3 x 6 for the lidar) | 1 | 12 approx. | Amazon | |
| 14 | M2.5 socket-head screw assortment | 1 | 12 approx. | Amazon | |
| 15 | M5 x 16 socket head + nyloc, 2 each | 1 | 3 approx. | hardware store | cable tidy edge bolts |
| 16 | VELCRO ONE-WRAP roll, 3/4 in (nearest to 25 mm; 1 in also exists) | 1 | 10 approx. | https://www.amazon.com/VELCRO-Brand-ONE-WRAP-Double-Sided-Multi-Purpose/dp/B000078CUB | VESC saddle straps (2 x 25 mm) |
| 17 | Baseplate v2 (new car), 1/8 in Baltic birch, laser cut from `out/baseplate_v2_newcar.dxf`, 452 x 177.5 | 1 | 25 approx. | https://sendcutsend.com/materials/baltic-birch-plywood/ (instant quote) | **cut only after the tower / post spacing is measured** (NEWCAR_LAYOUT.md section 8 item 3) |
| 18 | 5.5 x 2.5 mm DC female jack pigtails | 1 pack | 8 approx. | https://www.amazon.com/HTTX-10-Pack-Socket-Female-Adapter/dp/B06XNR1ZVL | the 15 V split point |
| 19 | 5.5 x 2.5 mm DC male plug pigtails | 1 pack | 8 approx. | https://www.amazon.com/Fancasee-Replacement-Degree-Pigtail-Supply/dp/B081TXY6ML | to the Jetson jack |
| 20 | zip ties 2.5 x 100 mm | 1 pack | 6 approx. | hardware store | plate zip holes |
| 21 | PLA/PETG for the two new prints + 14 feet (about 250 g) | | 6 approx. | | plinth 63 g solid / bank tray 102 g solid, less at 60 % infill |
| | **subtotal B** | | **383.93** | | |

## C. Conditional

| # | item | qty | unit | source | when |
| --- | --- | --- | --- | --- | --- |
| 22 | SICK 2095617 (YF2A15-020UB5XLEAX) M12 5-pin A-coded female, 2 m, open ends | 1 | 15.67 | https://www.tme.com/us/en-us/details/yf2a15-020ub5xleax/sensors-cables/sick/ | if the donor's power cable is not 5-pin |
| 23 | M12 4-pin D-coded male to RJ45, 1 m, shielded (HangTon) | 1 | 18 approx. | https://www.amazon.com/HangTon-Industrial-D-Coded-Ethernet-Shielded/dp/B0953QF7B4 | if the donor's Ethernet cable is the wrong coding or gender (SICK 6034414, 2 m, is about 77) |
| 24 | 4S 5000 mAh hard-case LiPo with XT90 | 1 | 65 approx. | e.g. smc-racing.com / gensace | if the old car's pack is not shared |
| 25 | Castle CC BEC 2.0 (010-0154-00), 4.75-12 V adjustable, 13 A peak | 1 | 46.95 | https://www.castlecreations.com/en/cc-bec-2-0-010-0154-00 | instead of #4 if the servo stalls the UBEC |
| | **B + cables (22, 23)** | | **417.60** | | |
| | **B + all of C** | | **529.55** | | |

## D. Not needed, and why

- USB hub: the devkit has four USB-A ports (two stacks, 3 A each); camera + VESC + a spare.
  The lidar is on Ethernet. So no small-board plate either.
- PCA9685: the VESC's PPM output drives the servo.
- Buck converter: the PD trigger delivers 15 V directly; the lidar accepts 9-28 V.
- A servo mount or adapter: the INJS235 is a drop-in for the 6830 bulkhead pocket
  (INJORA_SERVO_MOUNT.md). Use the chassis's own 6845X servo saver, not the INJORA horn.
- The 20 V trigger (Adafruit 5452): 20.25-20.5 V at the Jetson's 20 V limit.
- SICK mounting kits (2068398, 2086761): the plinth uses the sensor's own M3 threads.
