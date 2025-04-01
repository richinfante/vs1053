# VS1053 driver

This is a SPI driver for the generic **[VS1053 Breakout](http://www.vlsi.fi/en/products/vs1053.html)** by VLSI Solution.<br/>
A powerful Ogg Vorbis / MP3 / AAC / WMA / FLAC / MIDI Audio Codec chip.<br/>
Read more: [http://www.vlsi.fi/en/products/vs1053.html](http://www.vlsi.fi/en/products/vs1053.html).

## Example board:

![alt text](images/vs1053_front.png)

Based on https://github.com/baldram/ESP_VS1053_Library<br>
For detailed information please see [VS1053b Datasheet specification](http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf).

The device does not play nice with other devices on the same SPI bus.</br>
It needs full access of cs line to configure its mode.</br>
The SPI bus speed should be limited during the setup steps.</br>
So It is recommended to use a separate SPI bus if you have </br>
more devices that are connected on SPI bus

Decoder supports up to 24 bits, up to 48kHz<br>
For FLAC support the patches with FLAC support must be used

## Patches:
See [here](https://www.vlsi.fi/en/support/software/vs10xxpatches.html)
Only two are provided but others can be added if needed.

Patches must be loaded on startup and after a <code>soft_reset</code> is called</br>
<b>Note:</b> <code>set_mp3_mode</code> calls <code>soft_reset</code> so it must be called before.<br>
See [examples](https://gitlab.com/esp322054205/vs1053/tree/master/examples) for details


## Example:
For complete code please check [examples](https://gitlab.com/esp322054205/vs1053/tree/master/examples) folder.

For a bigger example you can also see my project here
https://gitlab.com/esp322054205/esp32s3-radio-vs1053