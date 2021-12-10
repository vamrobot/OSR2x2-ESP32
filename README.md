# OSR2x2-ESP32

Firmware for the OSR2x2

Some key features of the OSR2x2 and hence this firmware are:
 - Dual firmware modes:
     - Serial T-Code processing (normal) mode overlaid with manually controllable compression (off by default) and bend (off by default) movements
     - Auto stroker mode with manually controllable compression (off by default) and bend (off by default) movements
 - Compression mode/ability, driven by either T-Code 'A3' or the push buttons (for increasing and decreasinig it's frequency)
 - Bend mode/ability, driven by either T-Code 'A4' or the push buttons (for increasing and decreasinig it's frequency)
 - Ability to set a constant 'average' gap between the dual ring receivers, which allows the use of any number of sleeves like the
    Quickshots, the 'Chinese' (Aliexpress) Quickshots, TOMAX, tons of Onaholes, etc
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
