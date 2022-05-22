# OSR2x2-ESP32

Firmware for the OSR2x2

Some key features of the OSR2x2 and hence the two firmware versions are:
 - Dual firmware modes:
     - Serial T-Code processing (normal) mode overlaid with manually controllable compression and bend movements
     - Auto stroker mode with manually controllable compression (off by default) and bend (off by default) movements
 - Compression mode/ability, driven by either T-Code 'A3' or the potentiometers or the push buttons (for increasing and decreasinig it's frequency)
 - Bend mode/ability, driven by either T-Code 'A4' or the potentiometers or the push buttons (for increasing and decreasinig it's frequency)
 - There are two OSR2x2 variants, one that uses 6 potentiometers and one that uses 6 push buttons
     - For the OSR2x2 that uses 6 potentiometers (OSR2x2-POTs-ESP32), their functions are:
         1. Stroke frequency (also switches the firmware to T-Code processing mode when near 0 or 'OFF')
             - Increases/Decreases L0 (x axis) stroke frequency in manual mode, also switches firmware mode to T-Code processing when the potentiometer and hence L0 (x axis) stroke frequency is 0 (near), when the stroker speed is increased to anything about 0 then the firmware mode automatically switches back to manual mode.
         2. Compression frequency
             - Increases/Decreases A3 (compression) stroke frequency in manual mode and is overlaid upon the T-Code processing mode, also turns 'off' compression when the potentiometer and hence A3 (compression) stroke frequency is 0 (near)
         3. Bend frequency
             - Increases/Decreases A4 (bend) stroke frequency in manual mode and is overlaid upon the T-Code processing mode, also turns 'off' bend when the potentiometer and hence A4 (bend) stroke frequency is 0 (near)
         4. Distance between rings
             - Increases/Decreases the average distance between the rings in manual mode and is overlaid upon the T-Code processing mode, used to space the rings according to the Fleshlight, Onahole, or any other type of sleeve you are using
         5. Stroke range
             - Increases/Decreases the total L0 (x axis) stroke range in manual mode
         6. Stroke range center point
             - Raises(Increases)/Lowers(Decreases) the 'centerpoint' of the L0 (x axis) stroke range in manual mode 

     - For the OSR2x2 that uses 6 push buttons (OSR2x2-Buttons-ESP32), their functions are:
         1. Increase stoke frequency and switch firmware
             - Increase L0 (x axis) stroke frequency in manual mode and switches firmware modes if held for 2 seconds
         2. Decrease stroke frequency and switch buttons from stroke to ring spacing mode
             - Decrease L0 (x axis) stroke frequency in manual mode and switches the two push buttons that function as Increase and Decrease for L0 (x axis) to function as Increase and Decrease for the Dual Ring Receiver Offset if held for 2 seconds, and if you hold it again for 2 seconds it toggles the function of the two buttons back to functioning as Increase and Decrease for L0 (x axis).
         3. Increase compression frequency and compression toggle
             - Increase A3 (compression) stroke frequency in manual mode and enables/disables A3 (compression) if held for 2 seconds
         4. Decrease compression frequency
             - Decrease A3 (compression) stroke frequency in manual mode and if the button is held for 2 seconds then the Dual Ring Receiver Offset value is reset to OSR2X2_DUAL_RING_RECEIVER_OFFSET
         5. Increase bend frequency and bend toggle
             - Increase A4 (bend) stroke frequency in manual mode and enables/disables A4 (bend) if held for 2 seconds
         6. Decrease bend frequency
             - Decrease A4 (bend) stroke frequency in manual mode

 - Ability to set a constant 'average' gap between the dual ring receivers, which allows the use of any number of sleeves like the
    Quickshots, the 'Chinese' (Aliexpress) Quickshots, TOMAX, tons of Onaholes, etc. (Can also be controlled by the potentiometers or the push buttons, see the INOs for details)
      - Example:
        - // This allows you to bring the dual ring receivers closer (positive value) or
        - // spread the dual ring receivers apart (negative value)
        - // Example numbers to try for different sleeves:
        - // Quickshot - 0
        - // Quickshot compressed slightly - 20
        - // Chinese Quickshot - -20
        - // TOMAX Lilith Uterus - -90
        - // Absorption Lilith Six Uterus - -50
        - #define OSR2X2_DUAL_RING_RECEIVER_OFFSET 0 
- Ability to set the independent frequencies of the L0 (x axis) strokes, the A3 (compressions) and the A4 (bends) when in
    'standalone' auto stroker firmware mode
     - Example:
          - // Sets the frequencies for the different manual stroker operations
          - #define ManualL0FrequencyInitial 0.6 // Initial L0 (x axis) stroke frequency, in strokes per second
          - #define ManualL0FrequencyStep 0.2 // Step value up/down for the L0 (x axis) stroke frequency, in strokes per second
          - #define ManualA3FrequencyInitial 1 // Initial A3 (compression) stroke frequency, in strokes per second
          - #define ManualA3FrequencyStep 0.2 // Step value up/down for the A3 (compression) stroke frequency, in strokes per second
          - #define ManualA4FrequencyInitial 2 // Initial A4 (bend) stroke frequency, in strokes per second
          - #define ManualA4FrequencyStep 0.2 // Step value up/down for the A4 (bend) stroke frequency, in strokes per second
