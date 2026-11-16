/*
 * ssd1306_probe.ino - SSD1306 behaviour probe for the Arduboy
 *
 * Purpose: exercise SSD1306 features that MAME's ssd1306 device
 * (src/devices/video/ssd1306.cpp) does not emulate, or emulates
 * differently from the datasheet, so the same binary can be run on real
 * hardware and in MAME and the results compared.
 *
 * Build: Board "Arduino Leonardo" (or an Arduboy board package).
 *        Libraries: Adafruit SSD1306, Adafruit GFX, Adafruit BusIO.
 *
 * Controls
 *   On a test card:  A = run test, B/Left = previous card, Right = next card
 *   While running:   Up/Down = next/previous step, Left/Right = re-run step,
 *                    A = next test card, B = back to this test's card
 *
 * Every test starts from a known state (fullInit()). The reference image
 * drawn by ref() is: 1px border, page numbers 0-7 on the right edge
 * (one per 8-row page), a large "F" in the middle (asymmetric in both
 * axes), "BL" in the bottom-left corner, and a label in page 0.
 *
 * ---------------------------------------------------------------------
 * State of MAME's ssd1306.cpp as reviewed (Nov 2026), relevant to tests:
 *
 *   Rendered:        AE/AF, A4/A5, A6/A7, 81 (contrast as pen grey),
 *                    C0/C8 (vertical flip only).
 *   Stored, not drawn: 40-7F start line, D3 offset, A8 mux, A0/A1 seg remap,
 *                    DA COM pins, 26/27/29/2A/2E/2F scroll, A3 v-area.
 *   Stored, no effect: D5 (scan rate only computed at reset), D9, DB.
 *   Ignored:         8D charge pump (param swallowed).
 *   Not decoded:     23 fade/blink, D6 zoom, 2C/2D content scroll, AD IREF.
 *                    Unknown commands do NOT consume their parameters, so
 *                    the parameters get executed as commands.
 *   Parser bugs:     A8 collects 3 bytes instead of 2 (swallows the next
 *                    command). 29h/2Ah direction taken from bit 0, so 29h
 *                    (datasheet: right) is stored as left.
 *   Other:           data writes are dropped while any scroll is active;
 *                    20h mode 3 stored with no pointer advance;
 *                    20h resets the RAM pointers (datasheet is silent).
 *   Not testable here: status read (Arduboy SPI is write-only).
 * ---------------------------------------------------------------------
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Arduboy (production) wiring
#define PIN_CS     12   // PD6
#define PIN_DC      4   // PD4
#define PIN_RST     6   // PD7
#define PIN_A       7   // PE6
#define PIN_B       8   // PB4
#define PIN_UP     A0   // PF7
#define PIN_RIGHT  A1   // PF6
#define PIN_LEFT   A2   // PF5
#define PIN_DOWN   A3   // PF4

#define W SSD1306_WHITE

typedef const __FlashStringHelper *FStr;

// Declared before any function: the Arduino IDE inserts generated
// prototypes above the first function definition.
typedef void (*TestFn)(uint8_t step, uint8_t prev);
struct Test {
	const char *title;
	const char *desc;
	uint8_t steps;
	TestFn fn;
};

Adafruit_SSD1306 oled(128, 64, &SPI, PIN_DC, PIN_RST, PIN_CS, 8000000UL);
static SPISettings oledSPI(8000000UL, MSBFIRST, SPI_MODE0);

static bool extDirty = false;   // an extension command (23h/D6h) was used

// ---------------------------------------------------------------------
// Raw controller access (bypasses the library's buffer)
// ---------------------------------------------------------------------

static void xferBegin(bool isData)
{
	SPI.beginTransaction(oledSPI);
	digitalWrite(PIN_DC, isData ? HIGH : LOW);
	digitalWrite(PIN_CS, LOW);
}

static void xferEnd()
{
	digitalWrite(PIN_CS, HIGH);
	SPI.endTransaction();
}

static void sendCmds(const uint8_t *p, uint8_t n)
{
	xferBegin(false);
	while (n--)
		SPI.transfer(*p++);
	xferEnd();
}

// alternate bytes a,b,a,b... for n bytes (a == b for a solid fill)
static void dataFill(uint8_t a, uint8_t b, uint16_t n)
{
	xferBegin(true);
	for (uint16_t i = 0; i < n; i++)
		SPI.transfer((i & 1) ? b : a);
	xferEnd();
}

#define CMD(...) do { const uint8_t c_[] = { __VA_ARGS__ }; sendCmds(c_, sizeof(c_)); } while (0)

// Known state: matches Adafruit/Arduboy init, horizontal addressing,
// full window, no scroll. Does not touch RAM; callers redraw.
static void fullInit()
{
	CMD(0xE3, 0xE3, 0xE3,     // NOPs: absorb any half-finished command
	    0x2E,                 // scroll off
	    0xA4, 0xA6,           // follow RAM, normal polarity
	    0xD5, 0x80,           // clock: reset value
	    0xA8, 0x3F, 0xE3,     // mux 64. E3 pad: MAME's A8 eats one extra byte
	    0xD3, 0x00,           // offset 0
	    0x40,                 // start line 0
	    0x8D, 0x14,           // charge pump on
	    0x20, 0x00,           // horizontal addressing
	    0xA1, 0xC8,           // Arduboy orientation
	    0xDA, 0x12,           // alternative COM, no L/R remap
	    0x81, 0xCF,           // contrast
	    0xD9, 0xF1,           // precharge
	    0xDB, 0x40,           // VCOMH
	    0xA3, 0x00, 0x40,     // vertical scroll area: whole screen
	    0x21, 0x00, 0x7F,     // column window
	    0x22, 0x00, 0x07,     // page window
	    0xAF);
	if (extDirty) {
		CMD(0x23, 0x00,       // fade/blink off
		    0xD6, 0x00);      // zoom off
		extDirty = false;
	}
}

// 26h/27h: op, start page, interval, end page, E, F (E/F dummies 00/FF on SSD1306)
static void hscroll(uint8_t op, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f)
{
	CMD(0x2E, op, 0x00, b, c, d, e, f, 0x2F);
}

// 29h/2Ah: op, start page, interval, end page, vertical offset
static void vscroll(uint8_t op, uint8_t b, uint8_t c, uint8_t d, uint8_t e)
{
	CMD(0x2E, op, 0x00, b, c, d, e, 0x2F);
}

// ---------------------------------------------------------------------
// Reference images (drawn into the library buffer, sent with display())
// ---------------------------------------------------------------------

static void printLabel(FStr lbl, int16_t v)
{
	oled.print(lbl);
	if (v >= 0) {
		oled.print(' ');
		oled.print((uint8_t)v, HEX);
	}
}

static void ref(FStr lbl, int16_t v)
{
	oled.clearDisplay();
	oled.drawRect(0, 0, 128, 64, W);
	for (uint8_t y = 8; y < 64; y += 8)
		oled.drawFastHLine(1, y, 4, W);              // row ticks every page
	oled.setTextSize(1);
	for (uint8_t p = 0; p < 8; p++) {
		oled.setCursor(120, p * 8 + 1);
		oled.print(p);
	}
	oled.setCursor(7, 55);
	oled.print(F("BL"));
	oled.setTextSize(3);
	oled.setCursor(54, 20);
	oled.print('F');
	oled.setTextSize(1);
	oled.setCursor(7, 1);
	printLabel(lbl, v);
}

// Ruler for scroll-distance tests. Pages 0-1: static ruler. Pages 2-6:
// scrolled content (bar at col 0-2 plus big F). Page 7: static label.
static void ruler(FStr lbl, int16_t v)
{
	oled.clearDisplay();
	oled.setTextSize(1);
	for (uint8_t x = 0; x < 128; x += 4) {
		if (x % 16)
			oled.drawFastVLine(x, 12, 4, W);
		else
			oled.drawFastVLine(x, 8, 8, W);
	}
	oled.setCursor(0, 0);  oled.print(0);
	oled.setCursor(32, 0); oled.print(32);
	oled.setCursor(64, 0); oled.print(64);
	oled.setCursor(96, 0); oled.print(96);
	oled.fillRect(0, 16, 3, 40, W);
	oled.setTextSize(3);
	oled.setCursor(54, 24);
	oled.print('F');
	oled.setTextSize(1);
	oled.setCursor(0, 56);
	printLabel(lbl, v);
	oled.display();
}

// fullInit + reference image
static void base(FStr lbl, int16_t v)
{
	fullInit();
	ref(lbl, v);
	oled.display();
}

// fullInit + empty screen with label in page 7 (for addressing tests).
// Afterwards the horizontal-mode pointer has wrapped back to page 0 col 0.
static void blank(FStr lbl)
{
	fullInit();
	oled.clearDisplay();
	oled.setTextSize(1);
	oled.setCursor(0, 56);
	oled.print(lbl);
	oled.display();
}

// ---------------------------------------------------------------------
// Tests. Signature: (step, previous step or 0xFF on fresh entry/re-run)
// ---------------------------------------------------------------------

// 1. Baseline orientation reference.
static void tBaseline(uint8_t, uint8_t)
{
	base(F("Baseline"), -1);
}

// 2. Display off/on; RAM writes while off must land.
static void tOnOff(uint8_t s, uint8_t)
{
	switch (s) {
	case 0:
		base(F("AF on"), -1);
		break;
	case 1:
		base(F("AE off"), -1);
		CMD(0xAE);
		break;
	case 2:
		base(F("AE off"), -1);
		CMD(0xAE);
		delay(500);
		ref(F("NEW, written off"), -1);
		oled.display();
		CMD(0xAF);
		break;
	}
}

// 3. Contrast sweep. MAME uses contrast directly as the grey level, so
// 00 renders black; real panels still emit at 00.
static const uint8_t contrastVals[] = { 0x00, 0x01, 0x10, 0x7F, 0xCF, 0xFF };
static void tContrast(uint8_t s, uint8_t)
{
	base(F("81h"), contrastVals[s]);
	CMD(0x81, contrastVals[s]);
}

// 4. Entire-display-on vs inverse. Datasheet doesn't define A5+A7.
// MAME: A5 short-circuits rendering before inversion is applied.
static void tAllOn(uint8_t s, uint8_t)
{
	static const uint8_t a[] = { 0xA4, 0xA5, 0xA4, 0xA5 };
	static const uint8_t b[] = { 0xA6, 0xA6, 0xA7, 0xA7 };
	base(F("A4/5 A6/7 step"), s);
	CMD(a[s], b[s]);
}

// 5. COM scan direction. Datasheet: immediate effect. MAME implements
// this (as a whole-screen vertical flip), so this is a regression check.
static void tComScan(uint8_t s, uint8_t)
{
	switch (s) {
	case 0: base(F("C8 normal"), -1); break;
	case 1: base(F("C0 no rewrite"), -1); CMD(0xC0); break;
	case 2: base(F("C0 + rewrite"), -1); CMD(0xC0); oled.display(); break;
	}
}

// 6. Segment remap. Datasheet: "only affects subsequent data input", so
// step 1 should show no change and step 2 a mirror. Hardware reports
// differ; this pins it down. MAME never applies A0/A1.
static void tSegRemap(uint8_t s, uint8_t)
{
	switch (s) {
	case 0: base(F("A1 normal"), -1); break;
	case 1: base(F("A0 no rewrite"), -1); CMD(0xA0); break;
	case 2: base(F("A0 + rewrite"), -1); CMD(0xA0); oled.display(); break;
	case 3: base(F("A0rw, A1 no rw"), -1); CMD(0xA0); oled.display(); CMD(0xA1); break;
	case 4: base(F("A0rw, A1 + rw"), -1); CMD(0xA0); oled.display(); CMD(0xA1); oled.display(); break;
	}
}

// 7. Display start line (40h-7Fh). RAM row n maps to COM0, wrapping.
// MAME stores m_display_start_line but screen_update never reads it.
static const uint8_t startVals[] = { 0, 1, 8, 32, 63 };
static void tStartLine(uint8_t s, uint8_t)
{
	if (s < 5) {
		base(F("40h+"), startVals[s]);
		CMD((uint8_t)(0x40 | startVals[s]));
	} else {
		base(F("C0, 40h+"), 8);
		CMD(0xC0, 0x48);
	}
}

// 8. Display offset (D3h). Shifts the COM mapping; with mux 64 it looks
// like 40h but the two differ once mux < 64 (see test 9). Not drawn in MAME.
static const uint8_t offsetVals[] = { 0, 8, 32, 56 };
static void tOffset(uint8_t s, uint8_t)
{
	if (s < 4) {
		base(F("D3"), offsetVals[s]);
		CMD(0xD3, offsetVals[s]);
	} else {
		base(F("C0, D3"), 8);
		CMD(0xC0, 0xD3, 0x08);
	}
}

// 9. Multiplex ratio. Rows beyond N+1 are not driven. 0E is invalid and
// must be ignored. Steps 5/6 show start line vs offset with mux 32.
// Every A8 is padded with E3 so MAME's 3-byte parse doesn't desync.
static const uint8_t muxVals[] = { 0x3F, 0x2F, 0x1F, 0x0F, 0x0E };
static void tMux(uint8_t s, uint8_t)
{
	if (s < 5) {
		base(F("A8"), muxVals[s]);
		CMD(0xA8, muxVals[s], 0xE3);
	} else if (s == 5) {
		base(F("A8 1F, 40h+"), 16);
		CMD(0xA8, 0x1F, 0xE3, 0x50);
	} else {
		base(F("A8 1F, D3"), 16);
		CMD(0xA8, 0x1F, 0xE3, 0xD3, 0x10);
	}
}

// 10. MAME parser bug: case 0xA8 calls populate_fifo_until_n_bytes(data, 3),
// so the command after the mux parameter is swallowed. Step 0 should be
// inverted on hardware and is not in MAME; step 1 (with a NOP) is inverted
// in both.
static void tA8Parse(uint8_t s, uint8_t)
{
	if (s == 0) {
		base(F("A8 3F A7"), -1);
		CMD(0xA8, 0x3F, 0xA7);
	} else {
		base(F("A8 3F E3 A7"), -1);
		CMD(0xA8, 0x3F, 0xE3, 0xA7);
	}
}

// 11. COM pins hardware config. The Arduboy panel is wired for 12h;
// 02h interleaves/splits rows, A[5] (32h/22h) swaps the COM halves.
// MAME stores the bits but always renders sequentially.
static const uint8_t comPinVals[] = { 0x12, 0x02, 0x32, 0x22 };
static void tComPins(uint8_t s, uint8_t)
{
	base(F("DA"), comPinVals[s]);
	CMD(0xDA, comPinVals[s]);
}

// 12. Precharge periods. Changes brightness and the frame rate (K term).
static const uint8_t prechargeVals[] = { 0xF1, 0x22, 0x11, 0x1F, 0xFF };
static void tPrecharge(uint8_t s, uint8_t)
{
	base(F("D9"), prechargeVals[s]);
	CMD(0xD9, prechargeVals[s]);
}

// 13. VCOMH deselect level. 40h/70h are undocumented but used by many
// init sequences (Adafruit uses 40h).
static const uint8_t vcomhVals[] = { 0x00, 0x20, 0x30, 0x40, 0x70 };
static void tVcomh(uint8_t s, uint8_t)
{
	base(F("DB"), vcomhVals[s]);
	CMD(0xDB, vcomhVals[s]);
}

// 14. Charge pump. The Arduboy panel has no external VCC, so pump off
// should blank or nearly blank it. MAME ignores 8Dh.
static void tPump(uint8_t s, uint8_t)
{
	base(F("8D step"), s);
	if (s >= 1)
		CMD(0x8D, 0x10, 0xAF);
	if (s == 2) {
		delay(500);
		CMD(0x8D, 0x14, 0xAF);
	}
}

// 15. Page addressing: column wraps to the column start address (not 0),
// page does not advance. MAME matches the datasheet. Regression check.
static void tPageWrap(uint8_t, uint8_t)
{
	blank(F("pg mode wrap"));
	CMD(0x20, 0x02, 0xB3, 0x08, 0x17);   // page 3, column 0x78 = 120
	dataFill(0xFF, 0xFF, 8);
	dataFill(0x81, 0x81, 8);
}

// 16. Horizontal/vertical addressing inside a 0x21/0x22 window.
// 256 bytes fill the window; 32 more wrap to the window start.
static void tWindow(uint8_t s, uint8_t)
{
	if (s == 0) {
		blank(F("H window"));
		CMD(0x20, 0x00, 0x21, 32, 95, 0x22, 2, 5);
	} else {
		blank(F("V window"));
		CMD(0x20, 0x01, 0x21, 32, 95, 0x22, 2, 5);
	}
	dataFill(0xFF, 0xFF, 256);
	dataFill(0x81, 0x81, 32);
}

// 17. Commands documented as mode-specific. A 16-byte solid box shows
// where data landed.
//  0: H mode, B5h + column 64 commands (page-mode only). DS: ignored,
//     box at page 0 col 0. Honoured: page 5 col 64.
//  1: page mode, 21h/22h (H/V only). DS: ignored, box at page 2 col 0.
//     Honoured: page 4 col 64.
//  2: H mode, 200 bytes of a thin line, then 20h 00 again. If 20h doesn't
//     reset the pointer the box continues the line at page 1 col 72.
//     MAME resets the pointer on 20h, so the box goes to page 0 col 0.
static void tGated(uint8_t s, uint8_t)
{
	switch (s) {
	case 0:
		blank(F("H mode + B5h"));
		CMD(0x21, 0, 127, 0x22, 0, 7, 0xB5, 0x00, 0x14);
		break;
	case 1:
		blank(F("pg mode + 21/22"));
		CMD(0x20, 0x02, 0xB2, 0x00, 0x10, 0x21, 64, 127, 0x22, 4, 7);
		break;
	case 2:
		blank(F("20h re-sent"));
		CMD(0x21, 0, 127, 0x22, 0, 7);
		dataFill(0x18, 0x18, 200);
		CMD(0x20, 0x00);
		break;
	}
	dataFill(0xFF, 0xFF, 16);
}

// 18. Invalid addressing mode 11b. MAME stores it and never advances the
// pointer (all 64 bytes hit one byte). Hardware behaviour unknown.
static void tBadMode(uint8_t, uint8_t)
{
	blank(F("20h 03"));
	CMD(0x21, 0, 127, 0x22, 0, 7, 0x20, 0x03);
	dataFill(0x55, 0x55, 64);
}

// 19. Out-of-range address bits.
//  0: BBh in page mode: is bit 3 ignored (page 3) or the command (page 0)?
//     MAME: logged invalid, pointer unchanged (page 0).
//  1: 22h with pages 0A..0D: masked to 2..5?
//  2: 21h with A0..FF: masked to 0x20..0x7F?
static void tAddrBits(uint8_t s, uint8_t)
{
	switch (s) {
	case 0:
		blank(F("BBh pg mode"));
		CMD(0x20, 0x02, 0xB0, 0x00, 0x10, 0xBB);
		break;
	case 1:
		blank(F("22h 0A 0D"));
		CMD(0x21, 0, 127, 0x22, 0x0A, 0x0D);
		break;
	case 2:
		blank(F("21h A0 FF"));
		CMD(0x22, 0, 7, 0x21, 0xA0, 0xFF);
		break;
	}
	dataFill(0xFF, 0xFF, 16);
}

// 20. Right scroll with each interval encoding. The tables differ between
// SSD1306 and SSD1306B, so this also identifies the controller revision.
// MAME stores the setup but never renders scrolling.
static void tHScrollC(uint8_t s, uint8_t)
{
	base(F("26h C="), s);
	hscroll(0x26, 0, s, 7, 0x00, 0xFF);
}

// 21. Partial-page left scroll, single page, and an invalid range.
static void tHScrollPart(uint8_t s, uint8_t)
{
	switch (s) {
	case 0: base(F("27h pg2-5"), -1); hscroll(0x27, 2, 7, 5, 0x00, 0xFF); break;
	case 1: base(F("27h pg3"), -1);   hscroll(0x27, 3, 7, 3, 0x00, 0xFF); break;
	case 2: base(F("27h pg5..2"), -1); hscroll(0x27, 5, 7, 2, 0x00, 0xFF); break;
	}
}

// 22. After 2Eh the datasheet says RAM must be rewritten, implying the
// scroll engine moves RAM contents. Does the bar stay where it stopped?
static void tFreeze(uint8_t s, uint8_t)
{
	ruler(s ? F("500ms 2E +rw") : F("500ms 2E"), -1);
	hscroll(0x26, 2, 7, 6, 0x00, 0xFF);
	delay(500);
	CMD(0x2E);
	if (s)
		oled.display();
}

// 23. Frame rate vs D5h, measured with the scroll engine: 1 s at 2 frames
// per column, then freeze. Bar column = fps/2 (mod 128).
// Ffrm = Fosc / (div * K * mux), K = P1 + P2 + 50 = 66 with D9=F1.
// MAME: update_scan_rate() only runs at reset, and nothing scrolls anyway.
static const uint8_t clockVals[] = { 0x80, 0xF0, 0x00, 0x83, 0x8F };
static void tFrameRate(uint8_t s, uint8_t)
{
	ruler(F("D5"), clockVals[s]);
	CMD(0xD5, clockVals[s]);
	hscroll(0x26, 2, 7, 6, 0x00, 0xFF);
	delay(1000);
	CMD(0x2E);
}

// 24. RAM writes during an active scroll (datasheet: prohibited).
// MAME's raw_write() drops them. Step 1 deactivates first.
static void tWriteScroll(uint8_t s, uint8_t)
{
	base(s ? F("2E, then write") : F("write @scroll"), -1);
	hscroll(0x26, 0, 6, 3, 0x00, 0xFF);
	if (s)
		CMD(0x2E);
	CMD(0x21, 0, 127, 0x22, 4, 7);
	dataFill(0xAA, 0x55, 512);
}

// 25. 29h/2Ah with zero vertical offset is horizontal-only, which isolates
// direction. Datasheet: 29h = right, 2Ah = left. MAME takes "left" from
// bit 0, so it records 29h as left.
static void tVHDir(uint8_t s, uint8_t)
{
	switch (s) {
	case 0: base(F("29h E=0"), -1); vscroll(0x29, 0, 7, 7, 0); break;
	case 1: base(F("2Ah E=0"), -1); vscroll(0x2A, 0, 7, 7, 0); break;
	case 2: base(F("26h ref"), -1); hscroll(0x26, 0, 7, 7, 0x00, 0xFF); break;
	}
}

// 26. Continuous vertical scroll. Horizontal movement only affects pages
// B..D; vertical applies to the A3 area (whole screen here).
static void tVScroll(uint8_t s, uint8_t)
{
	switch (s) {
	case 0: base(F("29h pg0-7 E1"), -1); vscroll(0x29, 0, 6, 7, 0x01); break;
	case 1: base(F("29h pg0 E1"), -1);   vscroll(0x29, 0, 6, 0, 0x01); break;
	case 2: base(F("29h pg0 E3F"), -1);  vscroll(0x29, 0, 6, 0, 0x3F); break;
	case 3: base(F("2Ah pg0 E1"), -1);   vscroll(0x2A, 0, 6, 0, 0x01); break;
	}
}

// 27. Vertical scroll area: top fixed rows / scrolled rows.
static const uint8_t areaTop[] = { 16, 0, 16, 0 };
static const uint8_t areaRows[] = { 48, 32, 32, 64 };
static void tVArea(uint8_t s, uint8_t)
{
	base(F("A3 fixed"), areaTop[s]);
	CMD(0xA3, areaTop[s], areaRows[s]);
	vscroll(0x29, 0, 6, 0, 0x01);
}

// 28. On SSD1306B/SSD1315 the last two 26h bytes are start/end column; on
// the original SSD1306 they are dummies 00/FF. MAME logs a mismatch.
static void tColWindow(uint8_t s, uint8_t)
{
	if (s == 0) {
		base(F("26h E20 F5F"), -1);
		hscroll(0x26, 0, 7, 7, 0x20, 0x5F);
	} else {
		base(F("26h E00 FFF"), -1);
		hscroll(0x26, 0, 7, 7, 0x00, 0xFF);
	}
}

// 29. Content scroll (SSD1306B/SSD1315): one-shot, one column per command,
// moves RAM. Bytes: 2C/2D, 00, start page, 01, end page, start col, end col.
// Needs >= 2 frames between commands. On a chip (or MAME) that doesn't
// know 2Ch, the parameters run as commands; 7Fh is "start line 63", which
// shows up as a vertical jump on hardware.
static void tContent(uint8_t s, uint8_t prev)
{
	if (s == 0 || prev == 0xFF) {
		base(F("2Ch/2Dh"), s);
		return;
	}
	CMD((uint8_t)((s > prev) ? 0x2C : 0x2D), 0x00, 0x00, 0x01, 0x07, 0x00, 0x7F);
	delay(40);
}

// 30. Fade out / blink (datasheet rev 1.3+, SSD1306B). MAME: invalid, and
// the parameter is then executed (20h = addressing mode, 2Fh = activate
// scroll, 30h/3Fh invalid). fullInit's leading NOPs absorb the fallout.
static const uint8_t fadeVals[] = { 0x00, 0x20, 0x2F, 0x30, 0x3F };
static void tFade(uint8_t s, uint8_t)
{
	base(F("23h"), fadeVals[s]);
	CMD(0x23, fadeVals[s]);
	extDirty = true;
}

// 31. Zoom in (datasheet rev 1.3+, SSD1306B). Requires DA A[4]=1.
static void tZoom(uint8_t s, uint8_t)
{
	base(F("D6"), s);
	CMD(0xD6, s);
	extDirty = true;
}

// ---------------------------------------------------------------------
// Test table (cards: title <= 15 chars, up to 7 lines of <= 21 chars)
// ---------------------------------------------------------------------


#define TXT static const char

TXT tt01[] PROGMEM = "Baseline";
TXT td01[] PROGMEM =
	"Ref: border, page no.\n"
	"0-7 at right, big F,\n"
	"BL marks bottom-left.\n"
	"A:next  B:this card\n"
	"Up/Dn: step  L/R:\n"
	"rerun step. On cards:\n"
	"A run B/L prev R next";

TXT tt02[] PROGMEM = "AE/AF on/off";
TXT td02[] PROGMEM =
	"0 AF: on\n"
	"1 AE: all black\n"
	"2 AE, RAM rewritten\n"
	"  while off, then AF:\n"
	"  label says NEW\n"
	"MAME: implemented";

TXT tt03[] PROGMEM = "81h contrast";
TXT td03[] PROGMEM =
	"Up/Dn: 00 01 10 7F\n"
	"       CF FF\n"
	"HW: 00 still faintly\n"
	"lit; curve nonlinear\n"
	"MAME: pen = contrast\n"
	"grey, 00 = black";

TXT tt04[] PROGMEM = "A5/A7 all-on";
TXT td04[] PROGMEM =
	"0 A4 A6 normal\n"
	"1 A5 entire disp on\n"
	"2 A7 inverse\n"
	"3 A5 + A7: DS silent,\n"
	"  record HW result\n"
	"MAME: 3 = all on\n"
	"  (A5 bypasses A7)";

TXT tt05[] PROGMEM = "C0/C8 COM scan";
TXT td05[] PROGMEM =
	"0 C8 normal\n"
	"1 C0 only: DS says\n"
	"  flips at once\n"
	"2 C0 + RAM rewrite:\n"
	"  same as 1\n"
	"MAME: implemented";

TXT tt06[] PROGMEM = "A0/A1 seg remap";
TXT td06[] PROGMEM =
	"0 A1 normal\n"
	"1 A0, no rewrite: DS\n"
	"  = no change\n"
	"2 A0 + rewrite:mirror\n"
	"3 then A1, no rewrite\n"
	"4 A1 + rewrite\n"
	"MAME: never mirrors";

TXT tt07[] PROGMEM = "40h start line";
TXT td07[] PROGMEM =
	"Up/Dn: 0 1 8 32 63,\n"
	"  then C0 + 8\n"
	"Image rolls UP n rows\n"
	"with wrap (row n at\n"
	"top). C0: other way.\n"
	"MAME: 40h stored,\n"
	"  never drawn";

TXT tt08[] PROGMEM = "D3h offset";
TXT td08[] PROGMEM =
	"Up/Dn: 0 8 32 56,\n"
	"  then C0 + 8\n"
	"COM shift, image\n"
	"moves vertically\n"
	"with wrap. Cf. 40h.\n"
	"MAME: D3 stored,\n"
	"  never drawn";

TXT tt09[] PROGMEM = "A8h mux ratio";
TXT td09[] PROGMEM =
	"Up/Dn: 3F 2F 1F 0F\n"
	"  0E(bad) 1F+40h16\n"
	"  1F+D3 16\n"
	"Only N+1 rows lit,\n"
	"rest dark. 0E must be\n"
	"ignored (stay 64).\n"
	"MAME: always 64 rows";

TXT tt10[] PROGMEM = "A8h parse bug";
TXT td10[] PROGMEM =
	"0 A8 3F A7\n"
	"1 A8 3F E3 A7\n"
	"Both should be\n"
	"INVERTED on HW.\n"
	"MAME reads 2 params\n"
	"for A8 and eats the\n"
	"A7 in step 0.";

TXT tt11[] PROGMEM = "DAh COM pins";
TXT td11[] PROGMEM =
	"Up/Dn: 12 02 32 22\n"
	"12 = normal (alt.)\n"
	"02 sequential: rows\n"
	"  interleave/split\n"
	"32/22 L/R remap:\n"
	"  halves swap\n"
	"MAME: never drawn";

TXT tt12[] PROGMEM = "D9h precharge";
TXT td12[] PROGMEM =
	"Up/Dn: F1 22 11 1F FF\n"
	"Phase 1/2 lengths:\n"
	"brightness, ghosting\n"
	"and frame rate shift\n"
	"on HW.\n"
	"MAME: stored only,\n"
	"no rate recalc";

TXT tt13[] PROGMEM = "DBh VCOMH";
TXT td13[] PROGMEM =
	"Up/Dn: 00 20 30 40 70\n"
	"Deselect level:\n"
	"brightness changes\n"
	"on HW (40/70 are\n"
	"undocumented).\n"
	"MAME: stored only";

TXT tt14[] PROGMEM = "8Dh charge pump";
TXT td14[] PROGMEM =
	"0 8D 14 pump on\n"
	"1 8D 10 pump off:\n"
	"  dark or very dim\n"
	"  (no ext. VCC)\n"
	"2 off, 0.5s, on again\n"
	"MAME: 8D ignored,\n"
	"  stays lit";

TXT tt15[] PROGMEM = "Page-mode wrap";
TXT td15[] PROGMEM =
	"Pg mode, page 3, col\n"
	"120: 8x FF then 8x 81\n"
	"DS: col wraps to 120\n"
	"-> hollow box right\n"
	"edge only. Solid box\n"
	"+ lines at left edge\n"
	"= wrapped to col 0";

TXT tt16[] PROGMEM = "H/V window wrap";
TXT td16[] PROGMEM =
	"Window c32-95 pg2-5\n"
	"256x FF + 32x 81:\n"
	"0 H mode: hollow\n"
	"  strip top-left,\n"
	"  32 cols wide\n"
	"1 V mode: hollow\n"
	"  strip, 8 cols, 4 pg";

TXT tt17[] PROGMEM = "Mode-gated cmds";
TXT td17[] PROGMEM =
	"Box = where 16B land\n"
	"0 H mode, B5 c64: DS\n"
	"  top-left / pg5 c64\n"
	"1 Pg mode, 21h/22h:\n"
	"  DS pg2 c0 / pg4 c64\n"
	"2 resend 20h: box at\n"
	"  c72 pg1 = no reset";

TXT tt18[] PROGMEM = "20h mode 3";
TXT td18[] PROGMEM =
	"20h 03 is invalid.\n"
	"Then 64x 55 written\n"
	"from page 0, col 0.\n"
	"MAME: no advance, all\n"
	"  hit 1 byte\n"
	"HW: record where\n"
	"  they land";

TXT tt19[] PROGMEM = "Addr bit masks";
TXT td19[] PROGMEM =
	"0 Pg mode, BBh: pg3?\n"
	"  pg0 = ignored\n"
	"1 22h 0A 0D: pg2?\n"
	"2 21h A0 FF: col 32?\n"
	"MAME: 0 ignored,\n"
	"  1 and 2 masked";

TXT tt20[] PROGMEM = "26h interval";
TXT td20[] PROGMEM =
	"Right, all pages.\n"
	"Up/Dn C=0-7, frames:\n"
	"SSD1306:\n"
	"5 64 128 256 3 4 25 2\n"
	"SSD1306B:\n"
	"6 32 64 128 3 4 5 2\n"
	"MAME: never scrolls";

TXT tt21[] PROGMEM = "27h partial";
TXT td21[] PROGMEM =
	"0 Left, pages 2-5,\n"
	"  rest static\n"
	"1 Page 3 only\n"
	"2 Start>end (5..2):\n"
	"  invalid, record\n"
	"MAME: never scrolls";

TXT tt22[] PROGMEM = "2Eh freeze";
TXT td22[] PROGMEM =
	"Pg2-6 right 500ms,\n"
	"then 2Eh.\n"
	"0 Bar stays shifted?\n"
	"  (DS: RAM altered)\n"
	"1 + RAM rewrite: bar\n"
	"  back at col 0\n"
	"MAME: bar never moves";

TXT tt23[] PROGMEM = "D5h frame rate";
TXT td23[] PROGMEM =
	"D5, then 1s scroll at\n"
	"2 frames/col; frozen.\n"
	"Bar col = fps/2\n"
	"(mod 128) ~44 @80h.\n"
	"Up/Dn: 80 F0 00 83 8F\n"
	"MAME: no scroll and\n"
	"D5 not recalculated";

TXT tt24[] PROGMEM = "Write @ scroll";
TXT td24[] PROGMEM =
	"Pg0-3 scroll, then\n"
	"checker to pg4-7.\n"
	"DS: RAM access banned\n"
	"0 while active: HW?\n"
	"  MAME drops writes\n"
	"1 after 2Eh: checker\n"
	"  must appear";

TXT tt25[] PROGMEM = "29h/2Ah dir";
TXT td25[] PROGMEM =
	"E=0: horizontal only\n"
	"0 29h: RIGHT\n"
	"1 2Ah: LEFT\n"
	"2 26h ref: RIGHT\n"
	"MAME decodes 29h as\n"
	"left (bit 0 inverted)\n"
	"and never scrolls";

TXT tt26[] PROGMEM = "29h vertical";
TXT td26[] PROGMEM =
	"C=25 frames per step\n"
	"0 29h pg0-7 E=1: up\n"
	"  + right\n"
	"1 pg0 only: up, pg0\n"
	"  RAM moves right\n"
	"2 E=3F: down 1/step\n"
	"3 2Ah pg0 E=1";

TXT tt27[] PROGMEM = "A3h v-area";
TXT td27[] PROGMEM =
	"A3 fixed/scroll rows,\n"
	"29h E=1 (up):\n"
	"0 16/48 top 16 fixed\n"
	"1 0/32 top half moves\n"
	"2 16/32 middle band\n"
	"3 0/64 everything\n"
	"MAME: never drawn";

TXT tt28[] PROGMEM = "26h col window";
TXT td28[] PROGMEM =
	"26h E,F bytes: dummy\n"
	"00/FF on SSD1306;\n"
	"col start/end on\n"
	"SSD1306B/SSD1315.\n"
	"0 E=20 F=5F  1 00/FF\n"
	"MAME: logs bad dummy";

TXT tt29[] PROGMEM = "2Ch/2Dh content";
TXT td29[] PROGMEM =
	"Up: 2Ch 1 col right\n"
	"Dn: 2Dh 1 col left\n"
	"One-shot RAM move,\n"
	"SSD1306B/1315 only.\n"
	"Vertical jump = chip\n"
	"ran params as cmds\n"
	"MAME: invalid cmd";

TXT tt30[] PROGMEM = "23h fade/blink";
TXT td30[] PROGMEM =
	"Up/Dn: 00 off,\n"
	"20 2F fade out,\n"
	"30 3F blink\n"
	"(DS rev1.3+/SSD1306B)\n"
	"MAME: invalid; param\n"
	"runs as a command";

TXT tt31[] PROGMEM = "D6h zoom";
TXT td31[] PROGMEM =
	"0 D6 00 off\n"
	"1 D6 01: top 32 rows\n"
	"  doubled in height\n"
	"Needs DA=12 (set).\n"
	"DS rev1.3+/SSD1306B\n"
	"MAME: invalid cmd";

static const Test tests[] PROGMEM = {
	{ tt01, td01,  1, tBaseline    },
	{ tt02, td02,  3, tOnOff       },
	{ tt03, td03,  6, tContrast    },
	{ tt04, td04,  4, tAllOn       },
	{ tt05, td05,  3, tComScan     },
	{ tt06, td06,  5, tSegRemap    },
	{ tt07, td07,  6, tStartLine   },
	{ tt08, td08,  5, tOffset      },
	{ tt09, td09,  7, tMux         },
	{ tt10, td10,  2, tA8Parse     },
	{ tt11, td11,  4, tComPins     },
	{ tt12, td12,  5, tPrecharge   },
	{ tt13, td13,  5, tVcomh       },
	{ tt14, td14,  3, tPump        },
	{ tt15, td15,  1, tPageWrap    },
	{ tt16, td16,  2, tWindow      },
	{ tt17, td17,  3, tGated       },
	{ tt18, td18,  1, tBadMode     },
	{ tt19, td19,  3, tAddrBits    },
	{ tt20, td20,  8, tHScrollC    },
	{ tt21, td21,  3, tHScrollPart },
	{ tt22, td22,  2, tFreeze      },
	{ tt23, td23,  5, tFrameRate   },
	{ tt24, td24,  2, tWriteScroll },
	{ tt25, td25,  3, tVHDir       },
	{ tt26, td26,  4, tVScroll     },
	{ tt27, td27,  4, tVArea       },
	{ tt28, td28,  2, tColWindow   },
	{ tt29, td29, 16, tContent     },
	{ tt30, td30,  5, tFade        },
	{ tt31, td31,  2, tZoom        },
};
#define NTESTS (sizeof(tests) / sizeof(tests[0]))

// ---------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------

enum { BTN_A = 1, BTN_B = 2, BTN_UP = 4, BTN_DOWN = 8, BTN_LEFT = 16, BTN_RIGHT = 32 };

static uint8_t cur = 0;
static uint8_t step = 0;
static bool running = false;
static uint8_t lastBtn = 0xFF;   // treat buttons held at boot as already down

static void loadTest(uint8_t i, Test &t)
{
	memcpy_P(&t, &tests[i], sizeof(Test));
}

static uint8_t readButtons()
{
	uint8_t b = 0;
	if (!digitalRead(PIN_A))     b |= BTN_A;
	if (!digitalRead(PIN_B))     b |= BTN_B;
	if (!digitalRead(PIN_UP))    b |= BTN_UP;
	if (!digitalRead(PIN_DOWN))  b |= BTN_DOWN;
	if (!digitalRead(PIN_LEFT))  b |= BTN_LEFT;
	if (!digitalRead(PIN_RIGHT)) b |= BTN_RIGHT;
	return b;
}

static void showCard()
{
	Test t;
	loadTest(cur, t);
	running = false;
	fullInit();
	oled.clearDisplay();
	oled.setTextSize(1);
	oled.setCursor(0, 0);
	if (cur + 1 < 10)
		oled.print('0');
	oled.print(cur + 1);
	oled.print('/');
	oled.print((uint8_t)NTESTS);
	oled.print(' ');
	oled.print((FStr)t.title);
	oled.setCursor(0, 8);
	oled.print((FStr)t.desc);
	oled.display();
}

void setup()
{
	pinMode(PIN_A, INPUT_PULLUP);
	pinMode(PIN_B, INPUT_PULLUP);
	pinMode(PIN_UP, INPUT_PULLUP);
	pinMode(PIN_DOWN, INPUT_PULLUP);
	pinMode(PIN_LEFT, INPUT_PULLUP);
	pinMode(PIN_RIGHT, INPUT_PULLUP);

	if (!oled.begin(SSD1306_SWITCHCAPVCC))
		for (;;) { }          // buffer allocation failed

	oled.setTextColor(W);
	oled.setTextWrap(true);
	showCard();
}

void loop()
{
	uint8_t now = readButtons();
	uint8_t hit = now & ~lastBtn;
	lastBtn = now;

	if (hit) {
		Test t;
		loadTest(cur, t);
		if (!running) {
			if (hit & BTN_A) {
				running = true;
				step = 0;
				t.fn(0, 0xFF);
			} else if (hit & (BTN_B | BTN_LEFT)) {
				cur = cur ? cur - 1 : NTESTS - 1;
				showCard();
			} else if (hit & BTN_RIGHT) {
				cur = (cur + 1) % NTESTS;
				showCard();
			}
		} else {
			if (hit & BTN_A) {
				cur = (cur + 1) % NTESTS;
				showCard();
			} else if (hit & BTN_B) {
				showCard();
			} else if ((hit & BTN_UP) && step + 1 < t.steps) {
				uint8_t prev = step++;
				t.fn(step, prev);
			} else if ((hit & BTN_DOWN) && step > 0) {
				uint8_t prev = step--;
				t.fn(step, prev);
			} else if (hit & (BTN_LEFT | BTN_RIGHT)) {
				t.fn(step, 0xFF);
			}
		}
	}
	delay(20);
}
