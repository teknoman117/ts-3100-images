TS-3100 Flash Images
====================
A collection of Flash ROM images (created by me) for the TS-3100.

Flash Tool
----------
When replacing the RTC chips in the TS-3100, the random contents of the RAM on the CMOS chip caused
the embedded BIOS to think it had a different flash chip. As part of the user page cleanup it ended
up erasing the BIOS and DOS images on the board. Needed to write a flash tool that could run from
an external EEPROM so I could restore the contents of the flash.

As a bonus, I can now experiment with my own images on the device without fear of not being able to
restore them in the future.

