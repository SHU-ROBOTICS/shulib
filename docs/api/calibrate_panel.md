<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/calibrate_panel.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `calibrate_panel.hpp`

calibrate_panel — the calibration screen as a PURE MODEL.

This header declares **16** types (68 members), **14** free functions, and **61** constants.

Extracted from [`include/shulib/calibration/calibrate_panel.hpp`](../../include/shulib/calibration/calibrate_panel.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kLargeCols`](#klargecols) — *constant*
- [`kMediumCols`](#kmediumcols) — *constant*
- [`kSmallCols`](#ksmallcols) — *constant*
- [`kHeaderCols`](#kheadercols) — *constant*
- [`kNoteCols`](#knotecols) — *constant*
- [`kSmallHeight`](#ksmallheight) — *constant*
- [`kMediumHeight`](#kmediumheight) — *constant*
- [`kLargeHeight`](#klargeheight) — *constant*
- [`kPanelWidth`](#kpanelwidth) — *constant*
- [`kPanelLastRow`](#kpanellastrow) — *constant*
- [`kHitPadX`](#khitpadx) — *constant*
- [`kHitPadY`](#khitpady) — *constant*
- [`kArmingDelaySeconds`](#karmingdelayseconds) — *constant*
- [`struct PanelRect`](#struct-panelrect)
  - [`x0`](#panelrect-x0)
  - [`y0`](#panelrect-y0)
  - [`x1`](#panelrect-x1)
  - [`y1`](#panelrect-y1)
  - [`operator==`](#panelrect-operator-eq-eq)
- [`hitBox`](#hitbox) — *free function*
- [`contains`](#contains) — *free function*
- [`overlaps`](#overlaps) — *free function*
- [`within`](#within) — *free function*
- [`kHeaderBar`](#kheaderbar) — *constant*
- [`kPrimarySlot`](#kprimaryslot) — *constant*
- [`kAbortSlot`](#kabortslot) — *constant*
- [`kAbortRow`](#kabortrow) — *constant*
- [`kFrontBox`](#kfrontbox) — *constant*
- [`kBackBox`](#kbackbox) — *constant*
- [`kRunAgainBox`](#krunagainbox) — *constant*
- [`enum class TextSize`](#enum-class-textsize)
  - [`Small`](#textsize-small)
  - [`Medium`](#textsize-medium)
  - [`Large`](#textsize-large)
- [`budgetFor`](#budgetfor) — *free function*
- [`heightFor`](#heightfor) — *free function*
- [`enum class PanelTone`](#enum-class-paneltone)
  - [`Blue`](#paneltone-blue)
  - [`Amber`](#paneltone-amber)
  - [`Green`](#paneltone-green)
  - [`Red`](#paneltone-red)
  - [`Grey`](#paneltone-grey)
  - [`White`](#paneltone-white)
- [`enum class PanelScreen`](#enum-class-panelscreen)
  - [`NotStarted`](#panelscreen-notstarted)
  - [`ImuWait`](#panelscreen-imuwait)
  - [`Push`](#panelscreen-push)
  - [`WhichEnd`](#panelscreen-whichend)
  - [`Pulses`](#panelscreen-pulses)
  - [`Spin`](#panelscreen-spin)
  - [`Saved`](#panelscreen-saved)
  - [`Stopped`](#panelscreen-stopped)
- [`enum class SaveOutcome`](#enum-class-saveoutcome)
  - [`NotAttempted`](#saveoutcome-notattempted)
  - [`Saved`](#saveoutcome-saved)
  - [`NotSaved`](#saveoutcome-notsaved)
- [`enum class PanelButtonId`](#enum-class-panelbuttonid)
  - [`Done`](#panelbuttonid-done)
  - [`Front`](#panelbuttonid-front)
  - [`Back`](#panelbuttonid-back)
  - [`Abort`](#panelbuttonid-abort)
  - [`RunAgain`](#panelbuttonid-runagain)
- [`struct PanelButton`](#struct-panelbutton)
  - [`id`](#panelbutton-id)
  - [`rect`](#panelbutton-rect)
  - [`label`](#panelbutton-label)
  - [`armed`](#panelbutton-armed)
- [`kPanelLineCap`](#kpanellinecap) — *constant*
- [`kPanelMaxLines`](#kpanelmaxlines) — *constant*
- [`kPanelMaxButtons`](#kpanelmaxbuttons) — *constant*
- [`struct PanelLine`](#struct-panelline)
  - [`x`](#panelline-x)
  - [`y`](#panelline-y)
  - [`size`](#panelline-size)
  - [`tone`](#panelline-tone)
  - [`text`](#panelline-text)
- [`struct PanelView`](#struct-panelview)
  - [`screen`](#panelview-screen)
  - [`headerTone`](#panelview-headertone)
  - [`headerWord`](#panelview-headerword)
  - [`lines`](#panelview-lines)
  - [`lineCount`](#panelview-linecount)
  - [`buttons`](#panelview-buttons)
  - [`buttonCount`](#panelview-buttoncount)
- [`struct StoppedWords`](#struct-stoppedwords)
  - [`headline`](#stoppedwords-headline)
  - [`why`](#stoppedwords-why)
  - [`doLine`](#stoppedwords-doline)
- [`stoppedWords`](#stoppedwords) — *free function*
- [`screenText`](#screentext) — *free function*
- [`kHeaderIdle`](#kheaderidle) — *constant*
- [`kHeaderImu`](#kheaderimu) — *constant*
- [`kHeaderPush`](#kheaderpush) — *constant*
- [`kHeaderWhichEnd`](#kheaderwhichend) — *constant*
- [`kHeaderPulses`](#kheaderpulses) — *constant*
- [`kHeaderSpin`](#kheaderspin) — *constant*
- [`kHeaderSaved`](#kheadersaved) — *constant*
- [`kHeaderNotSaved`](#kheadernotsaved) — *constant*
- [`kHeaderMeasured`](#kheadermeasured) — *constant*
- [`kHeaderStopped`](#kheaderstopped) — *constant*
- [`kNotStarted`](#knotstarted) — *constant*
- [`kHandsOff`](#khandsoff) — *constant*
- [`kImuWarming`](#kimuwarming) — *constant*
- [`kPushForward`](#kpushforward) — *constant*
- [`kWhichEnd`](#kwhichend) — *constant*
- [`kStandClear`](#kstandclear) — *constant*
- [`kTwitching`](#ktwitching) — *constant*
- [`kSpinning`](#kspinning) — *constant*
- [`kSettling`](#ksettling) — *constant*
- [`kNextDoneWhenMoved`](#knextdonewhenmoved) — *constant*
- [`kNextLetGo`](#knextletgo) — *constant*
- [`kNextTouchDone`](#knexttouchdone) — *constant*
- [`kNextTwitch`](#knexttwitch) — *constant*
- [`kNextSpins`](#knextspins) — *constant*
- [`kNextSaves`](#knextsaves) — *constant*
- [`kSettlingHandsOff`](#ksettlinghandsoff) — *constant*
- [`kSpinAgain`](#kspinagain) — *constant*
- [`kSpinFirst`](#kspinfirst) — *constant*
- [`kCopyThenRunAgain`](#kcopythenrunagain) — *constant*
- [`kCopyNow`](#kcopynow) — *constant*
- [`kCopyThese`](#kcopythese) — *constant*
- [`kFullReasonOnSerial`](#kfullreasononserial) — *constant*
- [`kLabelDone`](#klabeldone) — *constant*
- [`kLabelFront`](#klabelfront) — *constant*
- [`kLabelBack`](#klabelback) — *constant*
- [`kLabelAbort`](#klabelabort) — *constant*
- [`kLabelRunAgain`](#klabelrunagain) — *constant*
- [`struct Checked`](#struct-checked)
  - [`text`](#checked-text)
  - [`cols`](#checked-cols)
- [`kAll`](#kall) — *constant*
- [`panelLiteralsFit`](#panelliteralsfit) — *free function*
- [`panelLayoutHolds`](#panellayoutholds) — *free function*
- [`struct Wrapped`](#struct-wrapped)
  - [`1]`](#wrapped-1)
  - [`count`](#wrapped-count)
  - [`complete`](#wrapped-complete)
- [`wrapWords`](#wrapwords) — *free function*
- [`viewFor`](#viewfor) — *free function*
- [`hitTest`](#hittest) — *free function*
- [`layoutKey`](#layoutkey) — *free function*
- [`struct TouchSample`](#struct-touchsample)
  - [`error`](#touchsample-error)
  - [`releaseCount`](#touchsample-releasecount)
  - [`x`](#touchsample-x)
  - [`y`](#touchsample-y)
- [`enum class TouchAction`](#enum-class-touchaction)
  - [`None`](#touchaction-none)
  - [`Answer`](#touchaction-answer)
  - [`RunAgain`](#touchaction-runagain)
- [`struct TouchResult`](#struct-touchresult)
  - [`action`](#touchresult-action)
  - [`answer`](#touchresult-answer)
  - [`hit`](#touchresult-hit)
  - [`newRelease`](#touchresult-newrelease)
- [`class TouchGate`](#class-touchgate)
  - [`TouchGate`](#touchgate-touchgate)
  - [`view`](#touchgate-view)
  - [`touch`](#touchgate-touch)

<a id="klargecols"></a>

## `kLargeCols`

```cpp
inline constexpr std::size_t kLargeCols = 18
```

Characters per line in the LARGE size. ESTIMATE, never measured on a V5 (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:73`](../../include/shulib/calibration/calibrate_panel.hpp#L73).*

<a id="kmediumcols"></a>

## `kMediumCols`

```cpp
inline constexpr std::size_t kMediumCols = 36
```

Characters per line in the MEDIUM size. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:75`](../../include/shulib/calibration/calibrate_panel.hpp#L75).*

<a id="ksmallcols"></a>

## `kSmallCols`

```cpp
inline constexpr std::size_t kSmallCols = 50
```

Characters per line in the SMALL size. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:77`](../../include/shulib/calibration/calibrate_panel.hpp#L77).*

<a id="kheadercols"></a>

## `kHeaderCols`

```cpp
inline constexpr std::size_t kHeaderCols = 24
```

The header word's own budget, MEDIUM: it shares the header bar with the build time the program prints at the right. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:80`](../../include/shulib/calibration/calibrate_panel.hpp#L80).*

<a id="knotecols"></a>

## `kNoteCols`

```cpp
inline constexpr std::size_t kNoteCols = 18
```

The note beside RUN AGAIN on the outcome screens, MEDIUM: the left half of the line only. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:83`](../../include/shulib/calibration/calibrate_panel.hpp#L83).*

<a id="ksmallheight"></a>

## `kSmallHeight`

```cpp
inline constexpr std::int16_t kSmallHeight = 16
```

The height a line of each size is laid out for, pixels. ESTIMATES, like the budgets (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:86`](../../include/shulib/calibration/calibrate_panel.hpp#L86).*

<a id="kmediumheight"></a>

## `kMediumHeight`

```cpp
inline constexpr std::int16_t kMediumHeight = 20
```

See kSmallHeight.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:88`](../../include/shulib/calibration/calibrate_panel.hpp#L88).*

<a id="klargeheight"></a>

## `kLargeHeight`

```cpp
inline constexpr std::int16_t kLargeHeight = 34
```

See kSmallHeight.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:90`](../../include/shulib/calibration/calibrate_panel.hpp#L90).*

<a id="kpanelwidth"></a>

## `kPanelWidth`

```cpp
inline constexpr std::int16_t kPanelWidth = 480
```

The screen's usable width, pixels.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:95`](../../include/shulib/calibration/calibrate_panel.hpp#L95).*

<a id="kpanellastrow"></a>

## `kPanelLastRow`

```cpp
inline constexpr std::int16_t kPanelLastRow = 235
```

The lowest row anything may be drawn on: VEXos keeps the rows below.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:97`](../../include/shulib/calibration/calibrate_panel.hpp#L97).*

<a id="khitpadx"></a>

## `kHitPadX`

```cpp
inline constexpr std::int16_t kHitPadX = 4
```

How far a touch may land outside a button, left/right, and still hit it — a resistive screen's release point wanders. Kept under half of every gap between two buttons, so a pad never makes two buttons overlap. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:101`](../../include/shulib/calibration/calibrate_panel.hpp#L101).*

<a id="khitpady"></a>

## `kHitPadY`

```cpp
inline constexpr std::int16_t kHitPadY = 10
```

How far a touch may land above or below a button and still hit it (the robot's capture has a release exactly on a button's top edge). ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:104`](../../include/shulib/calibration/calibrate_panel.hpp#L104).*

<a id="karmingdelayseconds"></a>

## `kArmingDelaySeconds`

```cpp
inline constexpr double kArmingDelaySeconds = 0.5
```

How long the new buttons stay greyed and dead after a layout change: long enough that a reflexive second tap lands on nothing, short enough not to feel broken. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:107`](../../include/shulib/calibration/calibrate_panel.hpp#L107).*

<a id="struct-panelrect"></a>

## `struct PanelRect`

```cpp
struct PanelRect
```

A rectangle in screen pixels with INCLUSIVE corners — PROS's own drawing convention.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:110`](../../include/shulib/calibration/calibrate_panel.hpp#L110).*

<a id="panelrect-x0"></a>

### `PanelRect::x0`

```cpp
std::int16_t x0 = 0
```

left column

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:111`](../../include/shulib/calibration/calibrate_panel.hpp#L111).*

<a id="panelrect-y0"></a>

### `PanelRect::y0`

```cpp
std::int16_t y0 = 0
```

top row

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:112`](../../include/shulib/calibration/calibrate_panel.hpp#L112).*

<a id="panelrect-x1"></a>

### `PanelRect::x1`

```cpp
std::int16_t x1 = 0
```

right column (inclusive)

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:113`](../../include/shulib/calibration/calibrate_panel.hpp#L113).*

<a id="panelrect-y1"></a>

### `PanelRect::y1`

```cpp
std::int16_t y1 = 0
```

bottom row (inclusive)

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:114`](../../include/shulib/calibration/calibrate_panel.hpp#L114).*

<a id="panelrect-operator-eq-eq"></a>

### `PanelRect::operator==`

```cpp
friend constexpr bool operator==(const PanelRect&, const PanelRect&) = default
```

Equal corners.

*function, declared at [`include/shulib/calibration/calibrate_panel.hpp:116`](../../include/shulib/calibration/calibrate_panel.hpp#L116).*

<a id="hitbox"></a>

## `hitBox`

```cpp
[[nodiscard]] constexpr PanelRect hitBox(PanelRect r) noexcept
```

The rectangle a touch must land in to hit a button drawn at `r`: padded kHitPadX sideways and kHitPadY up and down.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:121`](../../include/shulib/calibration/calibrate_panel.hpp#L121).*

<a id="contains"></a>

## `contains`

```cpp
[[nodiscard]] constexpr bool contains(PanelRect r, int x, int y) noexcept
```

True when (x, y) lies inside `r`, edges included.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:129`](../../include/shulib/calibration/calibrate_panel.hpp#L129).*

<a id="overlaps"></a>

## `overlaps`

```cpp
[[nodiscard]] constexpr bool overlaps(PanelRect a, PanelRect b) noexcept
```

True when `a` and `b` share at least one pixel.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:134`](../../include/shulib/calibration/calibrate_panel.hpp#L134).*

<a id="within"></a>

## `within`

```cpp
[[nodiscard]] constexpr bool within(PanelRect inner, PanelRect outer) noexcept
```

True when `inner` lies wholly inside `outer`.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:139`](../../include/shulib/calibration/calibrate_panel.hpp#L139).*

<a id="kheaderbar"></a>

## `kHeaderBar`

```cpp
inline constexpr PanelRect kHeaderBar{0, 0, 479, 26}
```

The header bar: its colour is the state (blue unpowered, amber powered, green saved, red stopped).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:146`](../../include/shulib/calibration/calibrate_panel.hpp#L146).*

<a id="kprimaryslot"></a>

## `kPrimarySlot`

```cpp
inline constexpr PanelRect kPrimarySlot{6, 180, 300, 234}
```

DONE's place, the primary slot at the bottom left.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:148`](../../include/shulib/calibration/calibrate_panel.hpp#L148).*

<a id="kabortslot"></a>

## `kAbortSlot`

```cpp
inline constexpr PanelRect kAbortSlot{330, 180, 474, 234}
```

ABORT's place in EVERY state that shows it — the bottom right. Never moves under a thumb.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:150`](../../include/shulib/calibration/calibrate_panel.hpp#L150).*

<a id="kabortrow"></a>

## `kAbortRow`

```cpp
inline constexpr PanelRect kAbortRow{6, 180, 474, 234}
```

ABORT in the two POWERED phases: the whole button row, still containing kAbortSlot.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:152`](../../include/shulib/calibration/calibrate_panel.hpp#L152).*

<a id="kfrontbox"></a>

## `kFrontBox`

```cpp
inline constexpr PanelRect kFrontBox{6, 72, 234, 158}
```

FRONT FIRST: a big box in the MIDDLE, never in the button row.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:154`](../../include/shulib/calibration/calibrate_panel.hpp#L154).*

<a id="kbackbox"></a>

## `kBackBox`

```cpp
inline constexpr PanelRect kBackBox{246, 72, 474, 158}
```

BACK FIRST: the same, on the right.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:156`](../../include/shulib/calibration/calibrate_panel.hpp#L156).*

<a id="krunagainbox"></a>

## `kRunAgainBox`

```cpp
inline constexpr PanelRect kRunAgainBox{246, 112, 474, 158}
```

RUN AGAIN on the outcome screens: middle right, clear of the button row.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:158`](../../include/shulib/calibration/calibrate_panel.hpp#L158).*

<a id="enum-class-textsize"></a>

## `enum class TextSize`

```cpp
enum class TextSize : std::uint8_t
```

The three text sizes PROS offers.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:161`](../../include/shulib/calibration/calibrate_panel.hpp#L161).*

<a id="textsize-small"></a>

### `TextSize::Small`

```cpp
Small
```

kSmallCols per line

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:162`](../../include/shulib/calibration/calibrate_panel.hpp#L162).*

<a id="textsize-medium"></a>

### `TextSize::Medium`

```cpp
Medium
```

kMediumCols per line

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:163`](../../include/shulib/calibration/calibrate_panel.hpp#L163).*

<a id="textsize-large"></a>

### `TextSize::Large`

```cpp
Large
```

kLargeCols per line

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:164`](../../include/shulib/calibration/calibrate_panel.hpp#L164).*

<a id="budgetfor"></a>

## `budgetFor`

```cpp
[[nodiscard]] constexpr std::size_t budgetFor(TextSize size) noexcept
```

The budget, in characters, of one line of `size`.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:168`](../../include/shulib/calibration/calibrate_panel.hpp#L168).*

<a id="heightfor"></a>

## `heightFor`

```cpp
[[nodiscard]] constexpr std::int16_t heightFor(TextSize size) noexcept
```

The height a line of `size` is laid out for, pixels (an ESTIMATE).

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:173`](../../include/shulib/calibration/calibrate_panel.hpp#L173).*

<a id="enum-class-paneltone"></a>

## `enum class PanelTone`

```cpp
enum class PanelTone : std::uint8_t
```

A colour by MEANING; the program maps each to RGB.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:180`](../../include/shulib/calibration/calibrate_panel.hpp#L180).*

<a id="paneltone-blue"></a>

### `PanelTone::Blue`

```cpp
Blue
```

waiting / unpowered (the header)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:181`](../../include/shulib/calibration/calibrate_panel.hpp#L181).*

<a id="paneltone-amber"></a>

### `PanelTone::Amber`

```cpp
Amber
```

powered — stand clear; or "not yet" on a number

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:182`](../../include/shulib/calibration/calibrate_panel.hpp#L182).*

<a id="paneltone-green"></a>

### `PanelTone::Green`

```cpp
Green
```

ready / saved / past the floor

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:183`](../../include/shulib/calibration/calibrate_panel.hpp#L183).*

<a id="paneltone-red"></a>

### `PanelTone::Red`

```cpp
Red
```

stopped / not saved / out of bounds

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:184`](../../include/shulib/calibration/calibrate_panel.hpp#L184).*

<a id="paneltone-grey"></a>

### `PanelTone::Grey`

```cpp
Grey
```

a greyed button, or a quiet line

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:185`](../../include/shulib/calibration/calibrate_panel.hpp#L185).*

<a id="paneltone-white"></a>

### `PanelTone::White`

```cpp
White
```

ordinary text

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:186`](../../include/shulib/calibration/calibrate_panel.hpp#L186).*

<a id="enum-class-panelscreen"></a>

## `enum class PanelScreen`

```cpp
enum class PanelScreen : std::uint8_t
```

What a screen is showing.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:190`](../../include/shulib/calibration/calibrate_panel.hpp#L190).*

<a id="panelscreen-notstarted"></a>

### `PanelScreen::NotStarted`

```cpp
NotStarted
```

the calibrator is Idle

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:191`](../../include/shulib/calibration/calibrate_panel.hpp#L191).*

<a id="panelscreen-imuwait"></a>

### `PanelScreen::ImuWait`

```cpp
ImuWait
```

hands off while the IMU starts

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:192`](../../include/shulib/calibration/calibrate_panel.hpp#L192).*

<a id="panelscreen-push"></a>

### `PanelScreen::Push`

```cpp
Push
```

push it forward along the tape

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:193`](../../include/shulib/calibration/calibrate_panel.hpp#L193).*

<a id="panelscreen-whichend"></a>

### `PanelScreen::WhichEnd`

```cpp
WhichEnd
```

FRONT or BACK first?

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:194`](../../include/shulib/calibration/calibrate_panel.hpp#L194).*

<a id="panelscreen-pulses"></a>

### `PanelScreen::Pulses`

```cpp
Pulses
```

powered: one motor at a time (AMBER)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:195`](../../include/shulib/calibration/calibrate_panel.hpp#L195).*

<a id="panelscreen-spin"></a>

### `PanelScreen::Spin`

```cpp
Spin
```

powered: the spin (AMBER)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:196`](../../include/shulib/calibration/calibrate_panel.hpp#L196).*

<a id="panelscreen-saved"></a>

### `PanelScreen::Saved`

```cpp
Saved
```

the run finished (SAVED / NOT SAVED / MEASURED)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:197`](../../include/shulib/calibration/calibrate_panel.hpp#L197).*

<a id="panelscreen-stopped"></a>

### `PanelScreen::Stopped`

```cpp
Stopped
```

the run was refused or aborted

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:198`](../../include/shulib/calibration/calibrate_panel.hpp#L198).*

<a id="enum-class-saveoutcome"></a>

## `enum class SaveOutcome`

```cpp
enum class SaveOutcome : std::uint8_t
```

Whether the program managed to write the result — the panel cannot know, so it is told.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:202`](../../include/shulib/calibration/calibrate_panel.hpp#L202).*

<a id="saveoutcome-notattempted"></a>

### `SaveOutcome::NotAttempted`

```cpp
NotAttempted
```

not written (yet): the header says MEASURED

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:203`](../../include/shulib/calibration/calibrate_panel.hpp#L203).*

<a id="saveoutcome-saved"></a>

### `SaveOutcome::Saved`

```cpp
Saved
```

written, flushed, read back and compared: SAVED

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:204`](../../include/shulib/calibration/calibrate_panel.hpp#L204).*

<a id="saveoutcome-notsaved"></a>

### `SaveOutcome::NotSaved`

```cpp
NotSaved
```

the write or the read-back failed: NOT SAVED, copy the numbers

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:205`](../../include/shulib/calibration/calibrate_panel.hpp#L205).*

<a id="enum-class-panelbuttonid"></a>

## `enum class PanelButtonId`

```cpp
enum class PanelButtonId : std::uint8_t
```

Which button.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:209`](../../include/shulib/calibration/calibrate_panel.hpp#L209).*

<a id="panelbuttonid-done"></a>

### `PanelButtonId::Done`

```cpp
Done
```

"I have finished pushing" — drawn only when pushReady()

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:210`](../../include/shulib/calibration/calibrate_panel.hpp#L210).*

<a id="panelbuttonid-front"></a>

### `PanelButtonId::Front`

```cpp
Front
```

FRONT FIRST

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:211`](../../include/shulib/calibration/calibrate_panel.hpp#L211).*

<a id="panelbuttonid-back"></a>

### `PanelButtonId::Back`

```cpp
Back
```

BACK FIRST

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:212`](../../include/shulib/calibration/calibrate_panel.hpp#L212).*

<a id="panelbuttonid-abort"></a>

### `PanelButtonId::Abort`

```cpp
Abort
```

stop now — always armed

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:213`](../../include/shulib/calibration/calibrate_panel.hpp#L213).*

<a id="panelbuttonid-runagain"></a>

### `PanelButtonId::RunAgain`

```cpp
RunAgain
```

on the outcome screens: the program rebuilds the calibrator

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:214`](../../include/shulib/calibration/calibrate_panel.hpp#L214).*

<a id="struct-panelbutton"></a>

## `struct PanelButton`

```cpp
struct PanelButton
```

One button of a view.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:218`](../../include/shulib/calibration/calibrate_panel.hpp#L218).*

<a id="panelbutton-id"></a>

### `PanelButton::id`

```cpp
PanelButtonId id = PanelButtonId::Abort
```

Which button.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:220`](../../include/shulib/calibration/calibrate_panel.hpp#L220).*

<a id="panelbutton-rect"></a>

### `PanelButton::rect`

```cpp
PanelRect rect{}
```

Where it is drawn (its hit box is hitBox(rect)).

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:222`](../../include/shulib/calibration/calibrate_panel.hpp#L222).*

<a id="panelbutton-label"></a>

### `PanelButton::label`

```cpp
const char* label = ""
```

Its label, MEDIUM, ASCII.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:224`](../../include/shulib/calibration/calibrate_panel.hpp#L224).*

<a id="panelbutton-armed"></a>

### `PanelButton::armed`

```cpp
bool armed = true
```

False while the arming delay runs: drawn GREYED and never hit.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:226`](../../include/shulib/calibration/calibrate_panel.hpp#L226).*

<a id="kpanellinecap"></a>

## `kPanelLineCap`

```cpp
inline constexpr std::size_t kPanelLineCap = kSmallCols + 1
```

The longest line a view carries, plus its terminator.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:230`](../../include/shulib/calibration/calibrate_panel.hpp#L230).*

<a id="kpanelmaxlines"></a>

## `kPanelMaxLines`

```cpp
inline constexpr std::size_t kPanelMaxLines = 10
```

The most text lines one view carries.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:232`](../../include/shulib/calibration/calibrate_panel.hpp#L232).*

<a id="kpanelmaxbuttons"></a>

## `kPanelMaxButtons`

```cpp
inline constexpr std::size_t kPanelMaxButtons = 3
```

The most buttons one view carries.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:234`](../../include/shulib/calibration/calibrate_panel.hpp#L234).*

<a id="struct-panelline"></a>

## `struct PanelLine`

```cpp
struct PanelLine
```

One line of text at a fixed place and size.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:237`](../../include/shulib/calibration/calibrate_panel.hpp#L237).*

<a id="panelline-x"></a>

### `PanelLine::x`

```cpp
std::int16_t x = 0
```

Left edge, pixels.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:239`](../../include/shulib/calibration/calibrate_panel.hpp#L239).*

<a id="panelline-y"></a>

### `PanelLine::y`

```cpp
std::int16_t y = 0
```

Top edge, pixels.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:241`](../../include/shulib/calibration/calibrate_panel.hpp#L241).*

<a id="panelline-size"></a>

### `PanelLine::size`

```cpp
TextSize size = TextSize::Medium
```

Its size; the text fits budgetFor(size) (the host tests check every reachable line).

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:243`](../../include/shulib/calibration/calibrate_panel.hpp#L243).*

<a id="panelline-tone"></a>

### `PanelLine::tone`

```cpp
PanelTone tone = PanelTone::White
```

Its colour, by meaning.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:245`](../../include/shulib/calibration/calibrate_panel.hpp#L245).*

<a id="panelline-text"></a>

### `PanelLine::text`

```cpp
char text[kPanelLineCap]{}
```

The text, NUL-terminated, ASCII, no '%'.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:247`](../../include/shulib/calibration/calibrate_panel.hpp#L247).*

<a id="struct-panelview"></a>

## `struct PanelView`

```cpp
struct PanelView
```

Everything one frame of the screen says, and which touches mean something.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:251`](../../include/shulib/calibration/calibrate_panel.hpp#L251).*

<a id="panelview-screen"></a>

### `PanelView::screen`

```cpp
PanelScreen screen = PanelScreen::NotStarted
```

What the screen is showing.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:253`](../../include/shulib/calibration/calibrate_panel.hpp#L253).*

<a id="panelview-headertone"></a>

### `PanelView::headerTone`

```cpp
PanelTone headerTone = PanelTone::Blue
```

The header bar's colour — the state at a glance.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:255`](../../include/shulib/calibration/calibrate_panel.hpp#L255).*

<a id="panelview-headerword"></a>

### `PanelView::headerWord`

```cpp
const char* headerWord = ""
```

The step word, left of the header bar, MEDIUM, at most kHeaderCols.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:257`](../../include/shulib/calibration/calibrate_panel.hpp#L257).*

<a id="panelview-lines"></a>

### `PanelView::lines`

```cpp
PanelLine lines[kPanelMaxLines]{}
```

The text lines, in drawing order.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:259`](../../include/shulib/calibration/calibrate_panel.hpp#L259).*

<a id="panelview-linecount"></a>

### `PanelView::lineCount`

```cpp
std::uint8_t lineCount = 0
```

How many entries of `lines` are real.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:261`](../../include/shulib/calibration/calibrate_panel.hpp#L261).*

<a id="panelview-buttons"></a>

### `PanelView::buttons`

```cpp
PanelButton buttons[kPanelMaxButtons]{}
```

The buttons (live or greyed), ABORT always last when present.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:263`](../../include/shulib/calibration/calibrate_panel.hpp#L263).*

<a id="panelview-buttoncount"></a>

### `PanelView::buttonCount`

```cpp
std::uint8_t buttonCount = 0
```

How many entries of `buttons` are real.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:265`](../../include/shulib/calibration/calibrate_panel.hpp#L265).*

<a id="struct-stoppedwords"></a>

## `struct StoppedWords`

```cpp
struct StoppedWords
```

The STOPPED screen's words for one refusal code.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:269`](../../include/shulib/calibration/calibrate_panel.hpp#L269).*

<a id="stoppedwords-headline"></a>

### `StoppedWords::headline`

```cpp
const char* headline
```

LARGE, at most kLargeCols.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:271`](../../include/shulib/calibration/calibrate_panel.hpp#L271).*

<a id="stoppedwords-why"></a>

### `StoppedWords::why`

```cpp
const char* why
```

MEDIUM, at most kMediumCols: what happened.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:273`](../../include/shulib/calibration/calibrate_panel.hpp#L273).*

<a id="stoppedwords-doline"></a>

### `StoppedWords::doLine`

```cpp
const char* doLine
```

MEDIUM, at most kMediumCols: what to do.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:275`](../../include/shulib/calibration/calibrate_panel.hpp#L275).*

<a id="stoppedwords"></a>

## `stoppedWords`

```cpp
[[nodiscard]] constexpr StoppedWords stoppedWords(Refusal code) noexcept
```

The STOPPED wording for `code` — keyed on the CODE, never on the reason's prose, which the screen shows below it in SMALL for the numbers. Every entry is checked at compile time.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:280`](../../include/shulib/calibration/calibrate_panel.hpp#L280).*

<a id="screentext"></a>

## `screenText`

```cpp
[[nodiscard]] constexpr bool screenText(std::string_view s, std::size_t cols) noexcept
```

True when `s` may be drawn in a line of `cols` characters: printable ASCII only, no '%', and no longer than `cols`. The compile-time gate for every literal below.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:329`](../../include/shulib/calibration/calibrate_panel.hpp#L329).*

<a id="kheaderidle"></a>

## `kHeaderIdle`

```cpp
inline constexpr std::string_view kHeaderIdle = "CALIBRATE"
```

Header words (kHeaderCols).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:346`](../../include/shulib/calibration/calibrate_panel.hpp#L346).*

<a id="kheaderimu"></a>

## `kHeaderImu`

```cpp
inline constexpr std::string_view kHeaderImu = "1/5 IMU"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:348`](../../include/shulib/calibration/calibrate_panel.hpp#L348).*

<a id="kheaderpush"></a>

## `kHeaderPush`

```cpp
inline constexpr std::string_view kHeaderPush = "2/5 PUSH"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:350`](../../include/shulib/calibration/calibrate_panel.hpp#L350).*

<a id="kheaderwhichend"></a>

## `kHeaderWhichEnd`

```cpp
inline constexpr std::string_view kHeaderWhichEnd = "3/5 WHICH END"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:352`](../../include/shulib/calibration/calibrate_panel.hpp#L352).*

<a id="kheaderpulses"></a>

## `kHeaderPulses`

```cpp
inline constexpr std::string_view kHeaderPulses = "4/5 PULSES"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:354`](../../include/shulib/calibration/calibrate_panel.hpp#L354).*

<a id="kheaderspin"></a>

## `kHeaderSpin`

```cpp
inline constexpr std::string_view kHeaderSpin = "5/5 SPIN"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:356`](../../include/shulib/calibration/calibrate_panel.hpp#L356).*

<a id="kheadersaved"></a>

## `kHeaderSaved`

```cpp
inline constexpr std::string_view kHeaderSaved = "SAVED"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:358`](../../include/shulib/calibration/calibrate_panel.hpp#L358).*

<a id="kheadernotsaved"></a>

## `kHeaderNotSaved`

```cpp
inline constexpr std::string_view kHeaderNotSaved = "NOT SAVED"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:360`](../../include/shulib/calibration/calibrate_panel.hpp#L360).*

<a id="kheadermeasured"></a>

## `kHeaderMeasured`

```cpp
inline constexpr std::string_view kHeaderMeasured = "MEASURED"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:362`](../../include/shulib/calibration/calibrate_panel.hpp#L362).*

<a id="kheaderstopped"></a>

## `kHeaderStopped`

```cpp
inline constexpr std::string_view kHeaderStopped = "STOPPED -- MOTORS OFF"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:364`](../../include/shulib/calibration/calibrate_panel.hpp#L364).*

<a id="knotstarted"></a>

## `kNotStarted`

```cpp
inline constexpr std::string_view kNotStarted = "NOT STARTED"
```

Instructions (kLargeCols).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:366`](../../include/shulib/calibration/calibrate_panel.hpp#L366).*

<a id="khandsoff"></a>

## `kHandsOff`

```cpp
inline constexpr std::string_view kHandsOff = "HANDS OFF"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:368`](../../include/shulib/calibration/calibrate_panel.hpp#L368).*

<a id="kimuwarming"></a>

## `kImuWarming`

```cpp
inline constexpr std::string_view kImuWarming = "IMU WARMING UP"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:370`](../../include/shulib/calibration/calibrate_panel.hpp#L370).*

<a id="kpushforward"></a>

## `kPushForward`

```cpp
inline constexpr std::string_view kPushForward = "PUSH IT FORWARD"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:372`](../../include/shulib/calibration/calibrate_panel.hpp#L372).*

<a id="kwhichend"></a>

## `kWhichEnd`

```cpp
inline constexpr std::string_view kWhichEnd = "WHICH END FIRST?"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:374`](../../include/shulib/calibration/calibrate_panel.hpp#L374).*

<a id="kstandclear"></a>

## `kStandClear`

```cpp
inline constexpr std::string_view kStandClear = "STAND CLEAR"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:376`](../../include/shulib/calibration/calibrate_panel.hpp#L376).*

<a id="ktwitching"></a>

## `kTwitching`

```cpp
inline constexpr std::string_view kTwitching = "TWITCHING MOTORS"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:378`](../../include/shulib/calibration/calibrate_panel.hpp#L378).*

<a id="kspinning"></a>

## `kSpinning`

```cpp
inline constexpr std::string_view kSpinning = "SPINNING"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:380`](../../include/shulib/calibration/calibrate_panel.hpp#L380).*

<a id="ksettling"></a>

## `kSettling`

```cpp
inline constexpr std::string_view kSettling = "SETTLING"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:382`](../../include/shulib/calibration/calibrate_panel.hpp#L382).*

<a id="knextdonewhenmoved"></a>

## `kNextDoneWhenMoved`

```cpp
inline constexpr std::string_view kNextDoneWhenMoved = "DONE shows once the motors turn"
```

NEXT / detail lines (kMediumCols).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:384`](../../include/shulib/calibration/calibrate_panel.hpp#L384).*

<a id="knextletgo"></a>

## `kNextLetGo`

```cpp
inline constexpr std::string_view kNextLetGo = "let go -- DONE shows when it stops"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:386`](../../include/shulib/calibration/calibrate_panel.hpp#L386).*

<a id="knexttouchdone"></a>

## `kNextTouchDone`

```cpp
inline constexpr std::string_view kNextTouchDone = "touch DONE -- then say which end"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:388`](../../include/shulib/calibration/calibrate_panel.hpp#L388).*

<a id="knexttwitch"></a>

## `kNextTwitch`

```cpp
inline constexpr std::string_view kNextTwitch = "NEXT: motors twitch -- stand clear"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:390`](../../include/shulib/calibration/calibrate_panel.hpp#L390).*

<a id="knextspins"></a>

## `kNextSpins`

```cpp
inline constexpr std::string_view kNextSpins = "NEXT: it spins -- stand clear"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:392`](../../include/shulib/calibration/calibrate_panel.hpp#L392).*

<a id="knextsaves"></a>

## `kNextSaves`

```cpp
inline constexpr std::string_view kNextSaves = "NEXT: it saves -- stay clear"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:394`](../../include/shulib/calibration/calibrate_panel.hpp#L394).*

<a id="ksettlinghandsoff"></a>

## `kSettlingHandsOff`

```cpp
inline constexpr std::string_view kSettlingHandsOff = "settling -- hands off"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:396`](../../include/shulib/calibration/calibrate_panel.hpp#L396).*

<a id="kspinagain"></a>

## `kSpinAgain`

```cpp
inline constexpr std::string_view kSpinAgain = "hands off -- it spins again"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:398`](../../include/shulib/calibration/calibrate_panel.hpp#L398).*

<a id="kspinfirst"></a>

## `kSpinFirst`

```cpp
inline constexpr std::string_view kSpinFirst = "hands off -- it spins next"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:400`](../../include/shulib/calibration/calibrate_panel.hpp#L400).*

<a id="kcopythenrunagain"></a>

## `kCopyThenRunAgain`

```cpp
inline constexpr std::string_view kCopyThenRunAgain = "NEXT: copy these, then RUN AGAIN to check"
```

Outcome lines (kSmallCols).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:402`](../../include/shulib/calibration/calibrate_panel.hpp#L402).*

<a id="kcopynow"></a>

## `kCopyNow`

```cpp
inline constexpr std::string_view kCopyNow = "NOT SAVED -- copy these numbers NOW"
```

See kCopyThenRunAgain.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:404`](../../include/shulib/calibration/calibrate_panel.hpp#L404).*

<a id="kcopythese"></a>

## `kCopyThese`

```cpp
inline constexpr std::string_view kCopyThese = "copy these numbers"
```

See kCopyThenRunAgain.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:406`](../../include/shulib/calibration/calibrate_panel.hpp#L406).*

<a id="kfullreasononserial"></a>

## `kFullReasonOnSerial`

```cpp
inline constexpr std::string_view kFullReasonOnSerial = "the full reason is on the serial log"
```

See kCopyThenRunAgain.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:408`](../../include/shulib/calibration/calibrate_panel.hpp#L408).*

<a id="klabeldone"></a>

## `kLabelDone`

```cpp
inline constexpr std::string_view kLabelDone = "DONE"
```

Button labels (MEDIUM; a label also fits its box: 12 characters).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:410`](../../include/shulib/calibration/calibrate_panel.hpp#L410).*

<a id="klabelfront"></a>

## `kLabelFront`

```cpp
inline constexpr std::string_view kLabelFront = "FRONT FIRST"
```

See kLabelDone.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:412`](../../include/shulib/calibration/calibrate_panel.hpp#L412).*

<a id="klabelback"></a>

## `kLabelBack`

```cpp
inline constexpr std::string_view kLabelBack = "BACK FIRST"
```

See kLabelDone.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:414`](../../include/shulib/calibration/calibrate_panel.hpp#L414).*

<a id="klabelabort"></a>

## `kLabelAbort`

```cpp
inline constexpr std::string_view kLabelAbort = "ABORT"
```

See kLabelDone.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:416`](../../include/shulib/calibration/calibrate_panel.hpp#L416).*

<a id="klabelrunagain"></a>

## `kLabelRunAgain`

```cpp
inline constexpr std::string_view kLabelRunAgain = "RUN AGAIN"
```

See kLabelDone.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:418`](../../include/shulib/calibration/calibrate_panel.hpp#L418).*

<a id="struct-checked"></a>

## `struct Checked`

```cpp
struct Checked
```

One literal and the budget it must fit.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:421`](../../include/shulib/calibration/calibrate_panel.hpp#L421).*

<a id="checked-text"></a>

### `Checked::text`

```cpp
std::string_view text
```

the literal

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:422`](../../include/shulib/calibration/calibrate_panel.hpp#L422).*

<a id="checked-cols"></a>

### `Checked::cols`

```cpp
std::size_t cols
```

its line's budget

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:423`](../../include/shulib/calibration/calibrate_panel.hpp#L423).*

<a id="kall"></a>

## `kAll`

```cpp
inline constexpr Checked kAll[] = { {kHeaderIdle, kHeaderCols}, {kHeaderImu, kHeaderCols}, {kHeaderPush, kHeaderCols}, {kHeaderWhichEnd, kHeaderCols}, {kHeaderPulses, kHeaderCols}, {kHeaderSpin, kHeaderCols}, {kHeaderSaved, kHeaderCols}, {kHeaderNotSaved, kHeaderCols}, {kHeaderMeasured, kHeaderCols}, {kHeaderStopped, kHeaderCols}, {kNotStarted, kLargeCols}, {kHandsOff, kLargeCols}, {kImuWarming, kLargeCols}, {kPushForward, kLargeCols}, {kWhichEnd, kLargeCols}, {kStandClear, kLargeCols}, {kTwitching, kLargeCols}, {kSpinning, kLargeCols}, {kSettling, kLargeCols}, {kNextDoneWhenMoved, kMediumCols}, {kNextLetGo, kMediumCols}, {kNextTouchDone, kMediumCols}, {kNextTwitch, kMediumCols}, {kNextSpins, kMediumCols}, {kNextSaves, kMediumCols}, {kSettlingHandsOff, kMediumCols}, {kSpinAgain, kMediumCols}, {kSpinFirst, kMediumCols}, {kCopyThenRunAgain, kSmallCols}, {kCopyNow, kSmallCols}, {kCopyThese, kSmallCols}, {kFullReasonOnSerial, kSmallCols}, {kLabelDone, 12}, {kLabelFront, 12}, {kLabelBack, 12}, {kLabelAbort, 12}, {kLabelRunAgain, 12}, }
```

EVERY literal above with its budget — the list the compile-time check walks.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:427`](../../include/shulib/calibration/calibrate_panel.hpp#L427).*

<a id="panelliteralsfit"></a>

## `panelLiteralsFit`

```cpp
[[nodiscard]] constexpr bool panelLiteralsFit() noexcept
```

True when every named literal and every STOPPED wording fits its budget and is ASCII with no '%' — evaluated at compile time below, so a budget change that breaks one fails the build.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:452`](../../include/shulib/calibration/calibrate_panel.hpp#L452).*

<a id="panellayoutholds"></a>

## `panelLayoutHolds`

```cpp
[[nodiscard]] constexpr bool panelLayoutHolds() noexcept
```

True when the fixed layout keeps its promises: everything on the screen above row 236; ABORT's slot inside the powered row; and no middle box (FRONT, BACK, RUN AGAIN) whose HIT box reaches the hit box of anything in the button row — so a second tap where DONE or ABORT was lands on nothing in the next layout. FRONT and BACK do not touch each other.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:475`](../../include/shulib/calibration/calibrate_panel.hpp#L475).*

<a id="struct-wrapped"></a>

## `struct Wrapped`

```cpp
template <std::size_t Cols, std::size_t Lines> struct Wrapped
```

Text broken into at most `Lines` lines of at most `Cols` characters — see wrapWords().

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:501`](../../include/shulib/calibration/calibrate_panel.hpp#L501).*

<a id="wrapped-1"></a>

### `Wrapped::1]`

```cpp
char line[Lines][Cols + 1]{}
```

The lines, each NUL-terminated.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:503`](../../include/shulib/calibration/calibrate_panel.hpp#L503).*

<a id="wrapped-count"></a>

### `Wrapped::count`

```cpp
std::size_t count = 0
```

How many lines are used.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:505`](../../include/shulib/calibration/calibrate_panel.hpp#L505).*

<a id="wrapped-complete"></a>

### `Wrapped::complete`

```cpp
bool complete = true
```

False when the text did not fit: a word longer than `Cols` (it is never split), or more text than `Lines` lines hold (the lines hold what fitted, in order; the rest is NOT shown and this flag says so).

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:509`](../../include/shulib/calibration/calibrate_panel.hpp#L509).*

<a id="wrapwords"></a>

## `wrapWords`

```cpp
template <std::size_t Cols, std::size_t Lines> [[nodiscard]] constexpr Wrapped<Cols, Lines> wrapWords(std::string_view text) noexcept
```

Break `text` into lines of at most `Cols` characters, ONLY at spaces: a word — and therefore a number like 1.37500 — is never split, runs of spaces collapse to one, and nothing is dropped silently (Wrapped::complete). constexpr, so a literal's wrapping can be checked at compile time. Allocation-free.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:517`](../../include/shulib/calibration/calibrate_panel.hpp#L517).*

<a id="viewfor"></a>

## `viewFor`

```cpp
[[nodiscard]] inline PanelView viewFor(const DrivetrainCalibrator& cal, double secondsSinceLayoutChange, SaveOutcome save = SaveOutcome::NotAttempted)
```

The view for this frame. `secondsSinceLayoutChange` arms the non-ABORT buttons once it reaches kArmingDelaySeconds (TouchGate::view() tracks it); `save` tells the finished screen whether the program managed to write the result. A pure function of the calibrator's public queries.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:657`](../../include/shulib/calibration/calibrate_panel.hpp#L657).*

<a id="hittest"></a>

## `hitTest`

```cpp
[[nodiscard]] constexpr int hitTest(const PanelView& v, int x, int y) noexcept
```

The index of the ARMED button whose hit box (hitBox()) contains (x, y), or −1. A greyed button is never hit; neither is anything outside every button.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:879`](../../include/shulib/calibration/calibrate_panel.hpp#L879).*

<a id="layoutkey"></a>

## `layoutKey`

```cpp
[[nodiscard]] constexpr std::uint32_t layoutKey(const PanelView& v) noexcept
```

A number that changes exactly when the LAYOUT does — the screen and every button's identity and rectangle — and NOT when a button merely arms. TouchGate restarts its arming clock on a change.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:890`](../../include/shulib/calibration/calibrate_panel.hpp#L890).*

<a id="struct-touchsample"></a>

## `struct TouchSample`

```cpp
struct TouchSample
```

One read of the touch screen, PROS-free: what screen_touch_status() returns, reduced to what a release-edge reader needs.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:913`](../../include/shulib/calibration/calibrate_panel.hpp#L913).*

<a id="touchsample-error"></a>

### `TouchSample::error`

```cpp
bool error = false
```

The read FAILED (PROS: E_TOUCH_ERROR, every field −1). Ignored — and it must not move the release count, or the next good read would replay the last touch.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:916`](../../include/shulib/calibration/calibrate_panel.hpp#L916).*

<a id="touchsample-releasecount"></a>

### `TouchSample::releaseCount`

```cpp
std::int32_t releaseCount = 0
```

PROS's release_count: it only ever increases, one per finger lifted.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:918`](../../include/shulib/calibration/calibrate_panel.hpp#L918).*

<a id="touchsample-x"></a>

### `TouchSample::x`

```cpp
std::int16_t x = 0
```

Where the LAST touch was — stale while idle, so it means something only on a new release.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:920`](../../include/shulib/calibration/calibrate_panel.hpp#L920).*

<a id="touchsample-y"></a>

### `TouchSample::y`

```cpp
std::int16_t y = 0
```

See x.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:922`](../../include/shulib/calibration/calibrate_panel.hpp#L922).*

<a id="enum-class-touchaction"></a>

## `enum class TouchAction`

```cpp
enum class TouchAction : std::uint8_t
```

What a touch should do.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:926`](../../include/shulib/calibration/calibrate_panel.hpp#L926).*

<a id="touchaction-none"></a>

### `TouchAction::None`

```cpp
None
```

nothing — no new release, a miss, a greyed button, or a stale layout

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:927`](../../include/shulib/calibration/calibrate_panel.hpp#L927).*

<a id="touchaction-answer"></a>

### `TouchAction::Answer`

```cpp
Answer
```

send TouchResult::answer to the calibrator

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:928`](../../include/shulib/calibration/calibrate_panel.hpp#L928).*

<a id="touchaction-runagain"></a>

### `TouchAction::RunAgain`

```cpp
RunAgain
```

rebuild the calibrator (the program's job — drivetrain_calibrator.hpp)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:929`](../../include/shulib/calibration/calibrate_panel.hpp#L929).*

<a id="struct-touchresult"></a>

## `struct TouchResult`

```cpp
struct TouchResult
```

The outcome of one touch read.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:933`](../../include/shulib/calibration/calibrate_panel.hpp#L933).*

<a id="touchresult-action"></a>

### `TouchResult::action`

```cpp
TouchAction action = TouchAction::None
```

What to do.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:935`](../../include/shulib/calibration/calibrate_panel.hpp#L935).*

<a id="touchresult-answer"></a>

### `TouchResult::answer`

```cpp
Answer answer = Answer::Abort
```

The answer, when action is Answer.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:937`](../../include/shulib/calibration/calibrate_panel.hpp#L937).*

<a id="touchresult-hit"></a>

### `TouchResult::hit`

```cpp
int hit = -1
```

The button index hit in the DRAWN view (−1 for none) — for the serial log.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:939`](../../include/shulib/calibration/calibrate_panel.hpp#L939).*

<a id="touchresult-newrelease"></a>

### `TouchResult::newRelease`

```cpp
bool newRelease = false
```

True when this read carried a NEW release (logged even when it hits nothing).

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:941`](../../include/shulib/calibration/calibrate_panel.hpp#L941).*

<a id="class-touchgate"></a>

## `class TouchGate`

```cpp
class TouchGate
```

The touch discipline, in one place: the release EDGE only; error reads ignored and never replayed; the layout clock that arms buttons kArmingDelaySeconds after every layout change; and a re-check against the calibrator before any answer — because the view a finger landed on may be one frame older than the state machine.

*class, declared at [`include/shulib/calibration/calibrate_panel.hpp:948`](../../include/shulib/calibration/calibrate_panel.hpp#L948).*

<a id="touchgate-touchgate"></a>

### `TouchGate::TouchGate`

```cpp
explicit TouchGate(std::int32_t releaseCountAtStart) noexcept
```

`releaseCountAtStart`: PROS's release_count when the panel first appears, so a tap made before the program existed is never replayed as an answer.

*function, declared at [`include/shulib/calibration/calibrate_panel.hpp:952`](../../include/shulib/calibration/calibrate_panel.hpp#L952).*

<a id="touchgate-view"></a>

### `TouchGate::view`

```cpp
[[nodiscard]] PanelView view(const DrivetrainCalibrator& cal, double nowSeconds, SaveOutcome save = SaveOutcome::NotAttempted)
```

The view to paint now. Restarts the arming clock at `nowSeconds` whenever the layout (layoutKey()) differs from the last view this returned — so the program never has to track it, and a new layout's buttons come up greyed on the very first frame they appear.

*function, declared at [`include/shulib/calibration/calibrate_panel.hpp:958`](../../include/shulib/calibration/calibrate_panel.hpp#L958).*

<a id="touchgate-touch"></a>

### `TouchGate::touch`

```cpp
[[nodiscard]] TouchResult touch(const TouchSample& s, const PanelView& drawn, const DrivetrainCalibrator& cal)
```

One touch read against the view that is ON THE SCREEN (`drawn`). A new release that lands on an armed button becomes an answer only if the calibrator still accepts it now (DONE also needs pushReady()); RUN AGAIN only once the run has ended.

*function, declared at [`include/shulib/calibration/calibrate_panel.hpp:974`](../../include/shulib/calibration/calibrate_panel.hpp#L974).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 57 lines, click to expand</summary>

```text

 calibrate_panel — the calibration screen as a PURE MODEL (chunk R3d Part 2F-1, 2026-09-24).

 ── Why this exists ─────────────────────────────────────────────────────────────────
 The first night the calibrator ran on robot two, the team lead's report on its screen was
 "I CANT READ IT AND THE BUTTONS DONT WORK I DONT KNOW AFTER I PUSH IT WHAT IM SUPPOSED TO DO".
 The capture showed every touch reaching the calibrator — the buttons worked; the SCREEN did
 not: the instruction was cut mid-sentence, the one number that mattered was small and grey,
 DONE was offered with nothing pushed (two of five runs ended that way), and FRONT FIRST
 appeared exactly where DONE had just been, so a reflexive second tap would silently answer
 the next question. None of that was testable, because all of it lived in a PROS-bound
 program. This header is the part of the screen that decides WHAT it says and WHICH touches
 mean something — so those decisions are unit-tested on the host rather than eyeballed on the
 robot. The program (src/calibrate_program.cpp, Part 2F-2) only RENDERS a PanelView.

 ── What it decides ─────────────────────────────────────────────────────────────────
  * viewFor(): per state, a header colour and word, an instruction of at most two LARGE lines,
    one big live number, a NEXT line, and the buttons that are LIVE, each with its rectangle.
  * The layout rules that make a mis-tap harmless, checked at COMPILE time on the constants and
    at test time on every reachable view:
      - ABORT sits in the SAME rectangle in every state that shows it; in the two POWERED
        phases it grows to the whole button row, which still contains that rectangle.
      - DONE is drawn only when DrivetrainCalibrator::pushReady() says so.
      - Questions (FRONT / BACK) and RUN AGAIN are boxes in the MIDDLE of the screen, never in
        the button row — so no button of a new layout lies under the button whose touch
        caused the change (the double-tap hazard).
      - After EVERY layout change the new non-ABORT buttons are drawn greyed and dead for
        kArmingDelaySeconds; ABORT is never delayed.
  * hitTest(): padded boxes; a greyed button never hits.
  * TouchGate: the release edge only, error reads ignored (and never replayed), the arming
    clock, and a re-check against the calibrator before any answer is sent.
  * The STOPPED wording per Refusal code — a LARGE headline, a WHY line and a DO line — so the
    screen explains a stop from a CODE, never by parsing prose.
  * wrapWords(): breaks text only at spaces — it never splits a word or a number, and it never
    drops text silently (it says when the text did not fit).

 ── What it does NOT decide ─────────────────────────────────────────────────────────
 Pixels a font lands on, colours' RGB values, the build stamp, redraw-only-what-changed, the
 SD card, serial output, RUN AGAIN's rebuild of the calibrator (a new DrivetrainCalibrator —
 drivetrain_calibrator.hpp says how), and the competition-state screens: all the program's.

 ── EVERY TEXT BUDGET IS AN ESTIMATE ────────────────────────────────────────────────
 The V5's three text sizes have never been measured in this project (Bench Tests has a SCREEN
 RULER for it). The budgets below — LARGE 18, MEDIUM 36, SMALL 50 characters per 472-pixel line
 — are conservative guesses, each ONE named constant, and every literal in this file is checked
 against them by static_assert: when the ruler is read, changing a constant is all it takes, and
 any text that no longer fits fails the build naming itself. The line heights and positions are
 estimates of the same kind.

 ── ASCII ONLY, AND NEVER A PERCENT SIGN ────────────────────────────────────────────
 PROS 4.2.2 draws text through VEXos's own display routines, whose fonts carry ASCII only, and
 it hands the already-formatted text to VEXos AS A FORMAT STRING — a '%' on screen is re-read as
 a conversion with no argument behind it. Every literal here is checked for both at compile
 time; every formatted line is checked by the host tests on every reachable view.

 PROS-free, allocation-free, no exceptions thrown; pure functions of the calibrator's public
 queries plus the time since the layout changed. NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
