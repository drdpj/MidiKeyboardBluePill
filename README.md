# MidiKeyboardBluePill
This is software for the STM32F103 Microcontroller to convert serial midi into the outputs that would be generated by the Hybrid Music 4000 keyboard for the BBC Micro and BBC Master.

A BluePill board, plus optocoupler circuit is all that is required to get this to work, however there is a complete hardware design too which incorporates MIDI-through and blinkenlights.

  Pins used: Userport PB0-7 (even pins 6-20) are on GPIOs PB4,6-12
  Userport CB1 (pin 2) -> Clk, GPIO PB13  (SPI2) - add external 10k pullup to 5V
  Userport CB2 (pin 4) -> MOSI, GPIO PB15 (SPI2)
  MIDI serial in (3.3V) -> GPIO PA3  (USART2)
  PA5 - Button (to ground) to program channel (press once and next note sets the channel, press twice and channel is ignored),
  PA4 - LED (active low, 560R to 3.3V) to indicate programming mode.

The circuit provided specifies a particular opto-isolator (which just happened to be the SMT part I had available) - you can use whatever circuit you like that fits with the midi 1.0 spec. I, and others, have had this working fine with a 6N138.

If you'd like to update the firmware on a pre-built unit, you'll need an st-link. If you have a nucleo development board, it has an ST-LINK V2 built in. Failing that, the very cheap £5 ones work fine (e.g. https://www.amazon.co.uk/DAOKAI-Downloader-Programmer-STM8STM32-Programming/dp/B09WVQNFNM), but they need to be used in conjunction with ST-LINK Utility rather than CubeProgrammer. It's available here: https://www.st.com/en/development-tools/stsw-link004.html

Connect GND to GND, SWCLK to SWCLK, SWDIO to SWDIO. Don't connect ANYTHING to +3.3V. 

<img width="297" alt="image" src="https://user-images.githubusercontent.com/2575676/213863482-8e0c1186-f947-4b91-9c7d-d3f016c662ef.png">
n.b. This is the "debug" header on the recent boards, pin 1 is the right hand pin of the group of 4 if the text is the right way up.

Connect +5V to a +5V header if there is one on your board, or Pin 1 or 3 of the user port connector:
<img width="428" alt="image" src="https://user-images.githubusercontent.com/2575676/213863636-4d92972e-64f8-43ad-af81-30086ec84a98.png">

Download the latest release bin file, load into the ST-Link software, and then use the program option to re-flash and verify the unit. There are numerous guides to using the ST-Link software on the internet, so I won't repeat them here - a quick google should reveal them.

Presently I have a few standalone, fully built and tested units for sale. Get in touch if you're interested.
![midi2m4k](https://user-images.githubusercontent.com/2575676/213465252-ee0512f1-578f-4770-ad0d-df5117150859.jpg)
