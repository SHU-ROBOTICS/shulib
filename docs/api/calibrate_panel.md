<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/calibrate_panel.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `calibrate_panel.hpp`

calibrate_panel — the calibration screen as a PURE MODEL.

This header declares **19** types (81 members), **14** free functions, and **81** constants.

Extracted from [`include/shulib/calibration/calibrate_panel.hpp`](../../include/shulib/calibration/calibrate_panel.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

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
- [`kChoiceLeftBox`](#kchoiceleftbox) — *constant*
- [`kChoiceRightBox`](#kchoicerightbox) — *constant*
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
  - [`Pulses`](#panelscreen-pulses)
  - [`Spin`](#panelscreen-spin)
  - [`Saved`](#panelscreen-saved)
  - [`Stopped`](#panelscreen-stopped)
  - [`Held`](#panelscreen-held)
- [`enum class SaveOutcome`](#enum-class-saveoutcome)
  - [`NotAttempted`](#saveoutcome-notattempted)
  - [`Saved`](#saveoutcome-saved)
  - [`NotSaved`](#saveoutcome-notsaved)
  - [`Held`](#saveoutcome-held)
- [`enum class FrontChoice`](#enum-class-frontchoice)
  - [`None`](#frontchoice-none)
  - [`New`](#frontchoice-new)
- [`enum class HoldReason`](#enum-class-holdreason)
  - [`None`](#holdreason-none)
  - [`FrontFlipped`](#holdreason-frontflipped)
  - [`MechanismTurned`](#holdreason-mechanismturned)
- [`struct FinishFacts`](#struct-finishfacts)
  - [`front`](#finishfacts-front)
  - [`hold`](#finishfacts-hold)
  - [`holdPort`](#finishfacts-holdport)
  - [`holdLine1`](#finishfacts-holdline1)
  - [`holdLine2`](#finishfacts-holdline2)
  - [`note`](#finishfacts-note)
  - [`typedPorts`](#finishfacts-typedports)
- [`enum class PanelButtonId`](#enum-class-panelbuttonid)
  - [`Done`](#panelbuttonid-done)
  - [`SaveHeld`](#panelbuttonid-saveheld)
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
- [`kHeaderPulses`](#kheaderpulses) — *constant*
- [`kHeaderSpin`](#kheaderspin) — *constant*
- [`kHeaderSaved`](#kheadersaved) — *constant*
- [`kHeaderNotSaved`](#kheadernotsaved) — *constant*
- [`kHeaderMeasured`](#kheadermeasured) — *constant*
- [`kHeaderStopped`](#kheaderstopped) — *constant*
- [`kNotStarted`](#knotstarted) — *constant*
- [`kHandsOff`](#khandsoff) — *constant*
- [`kImuWarming`](#kimuwarming) — *constant*
- [`kStandClear`](#kstandclear) — *constant*
- [`kTwitching`](#ktwitching) — *constant*
- [`kSpinning`](#kspinning) — *constant*
- [`kSettling`](#ksettling) — *constant*
- [`kFindingLeft`](#kfindingleft) — *constant*
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
- [`kLabelAbort`](#klabelabort) — *constant*
- [`kLabelRunAgain`](#klabelrunagain) — *constant*
- [`kHeaderFlipped`](#kheaderflipped) — *constant*
- [`kHeaderMechTurned`](#kheadermechturned) — *constant*
- [`kHeaderSavedNew`](#kheadersavednew) — *constant*
- [`kPushFrontFirst`](#kpushfrontfirst) — *constant*
- [`kNothingSavedYet`](#knothingsavedyet) — *constant*
- [`kFrontRule`](#kfrontrule) — *constant*
- [`kFlipWhat`](#kflipwhat) — *constant*
- [`kFlipAgain`](#kflipagain) — *constant*
- [`kFlipNew`](#kflipnew) — *constant*
- [`kFlipWarn`](#kflipwarn) — *constant*
- [`kMechNotOnCard`](#kmechnotoncard) — *constant*
- [`kMechSave`](#kmechsave) — *constant*
- [`kMechAgain`](#kmechagain) — *constant*
- [`kLabelNew`](#klabelnew) — *constant*
- [`kLabelSave`](#klabelsave) — *constant*
- [`kNoteFirst`](#knotefirst) — *constant*
- [`kNoteNotChecked`](#knotenotchecked) — *constant*
- [`kWarningCut`](#kwarningcut) — *constant*
- [`kFlipWhatTyped`](#kflipwhattyped) — *constant*
- [`kFlipNewTyped`](#kflipnewtyped) — *constant*
- [`kFlipWarnTyped`](#kflipwarntyped) — *constant*
- [`kNoteTypedSame`](#knotetypedsame) — *constant*
- [`kNoteFixTyped`](#knotefixtyped) — *constant*
- [`kNoteTypedPartial`](#knotetypedpartial) — *constant*
- [`struct Checked`](#struct-checked)
  - [`text`](#checked-text)
  - [`cols`](#checked-cols)
- [`kAll`](#kall) — *constant*
- [`panelLiteralsFit`](#panelliteralsfit) — *free function*
- [`panelLayoutHolds`](#panellayoutholds) — *free function*
- [`struct Wrapped`](#struct-wrapped)
  - [`line`](#wrapped-line)
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
  - [`SaveHeld`](#touchaction-saveheld)
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

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:96`](../../include/shulib/calibration/calibrate_panel.hpp#L96).*

<a id="kmediumcols"></a>

## `kMediumCols`

```cpp
inline constexpr std::size_t kMediumCols = 36
```

Characters per line in the MEDIUM size. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:98`](../../include/shulib/calibration/calibrate_panel.hpp#L98).*

<a id="ksmallcols"></a>

## `kSmallCols`

```cpp
inline constexpr std::size_t kSmallCols = 50
```

Characters per line in the SMALL size. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:100`](../../include/shulib/calibration/calibrate_panel.hpp#L100).*

<a id="kheadercols"></a>

## `kHeaderCols`

```cpp
inline constexpr std::size_t kHeaderCols = 24
```

The header word's own budget, MEDIUM: it shares the header bar with the build time the program prints at the right. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:103`](../../include/shulib/calibration/calibrate_panel.hpp#L103).*

<a id="knotecols"></a>

## `kNoteCols`

```cpp
inline constexpr std::size_t kNoteCols = 18
```

The note beside RUN AGAIN on the outcome screens, MEDIUM: the left half of the line only. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:106`](../../include/shulib/calibration/calibrate_panel.hpp#L106).*

<a id="ksmallheight"></a>

## `kSmallHeight`

```cpp
inline constexpr std::int16_t kSmallHeight = 16
```

The height a line of each size is laid out for, pixels. ESTIMATES, like the budgets (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:109`](../../include/shulib/calibration/calibrate_panel.hpp#L109).*

<a id="kmediumheight"></a>

## `kMediumHeight`

```cpp
inline constexpr std::int16_t kMediumHeight = 20
```

See kSmallHeight.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:111`](../../include/shulib/calibration/calibrate_panel.hpp#L111).*

<a id="klargeheight"></a>

## `kLargeHeight`

```cpp
inline constexpr std::int16_t kLargeHeight = 34
```

See kSmallHeight.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:113`](../../include/shulib/calibration/calibrate_panel.hpp#L113).*

<a id="kpanelwidth"></a>

## `kPanelWidth`

```cpp
inline constexpr std::int16_t kPanelWidth = 480
```

The screen's usable width, pixels.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:118`](../../include/shulib/calibration/calibrate_panel.hpp#L118).*

<a id="kpanellastrow"></a>

## `kPanelLastRow`

```cpp
inline constexpr std::int16_t kPanelLastRow = 235
```

The lowest row anything may be drawn on: VEXos keeps the rows below.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:120`](../../include/shulib/calibration/calibrate_panel.hpp#L120).*

<a id="khitpadx"></a>

## `kHitPadX`

```cpp
inline constexpr std::int16_t kHitPadX = 4
```

How far a touch may land outside a button, left/right, and still hit it — a resistive screen's release point wanders. Kept under half of every gap between two buttons, so a pad never makes two buttons overlap. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:124`](../../include/shulib/calibration/calibrate_panel.hpp#L124).*

<a id="khitpady"></a>

## `kHitPadY`

```cpp
inline constexpr std::int16_t kHitPadY = 10
```

How far a touch may land above or below a button and still hit it (the robot's capture has a release exactly on a button's top edge). ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:127`](../../include/shulib/calibration/calibrate_panel.hpp#L127).*

<a id="karmingdelayseconds"></a>

## `kArmingDelaySeconds`

```cpp
inline constexpr double kArmingDelaySeconds = 0.5
```

How long the new buttons stay greyed and dead after a layout change: long enough that a reflexive second tap lands on nothing, short enough not to feel broken. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:130`](../../include/shulib/calibration/calibrate_panel.hpp#L130).*

<a id="struct-panelrect"></a>

## `struct PanelRect`

```cpp
struct PanelRect
```

A rectangle in screen pixels with INCLUSIVE corners — PROS's own drawing convention.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:133`](../../include/shulib/calibration/calibrate_panel.hpp#L133).*

<a id="panelrect-x0"></a>

### `PanelRect::x0`

```cpp
std::int16_t x0 = 0
```

left column

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:134`](../../include/shulib/calibration/calibrate_panel.hpp#L134).*

<a id="panelrect-y0"></a>

### `PanelRect::y0`

```cpp
std::int16_t y0 = 0
```

top row

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:135`](../../include/shulib/calibration/calibrate_panel.hpp#L135).*

<a id="panelrect-x1"></a>

### `PanelRect::x1`

```cpp
std::int16_t x1 = 0
```

right column (inclusive)

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:136`](../../include/shulib/calibration/calibrate_panel.hpp#L136).*

<a id="panelrect-y1"></a>

### `PanelRect::y1`

```cpp
std::int16_t y1 = 0
```

bottom row (inclusive)

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:137`](../../include/shulib/calibration/calibrate_panel.hpp#L137).*

<a id="panelrect-operator-eq-eq"></a>

### `PanelRect::operator==`

```cpp
friend constexpr bool operator==(const PanelRect&, const PanelRect&) = default
```

Equal corners.

*function, declared at [`include/shulib/calibration/calibrate_panel.hpp:139`](../../include/shulib/calibration/calibrate_panel.hpp#L139).*

<a id="hitbox"></a>

## `hitBox`

```cpp
[[nodiscard]] constexpr PanelRect hitBox(PanelRect r) noexcept
```

The rectangle a touch must land in to hit a button drawn at `r`: padded kHitPadX sideways and kHitPadY up and down.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:144`](../../include/shulib/calibration/calibrate_panel.hpp#L144).*

<a id="contains"></a>

## `contains`

```cpp
[[nodiscard]] constexpr bool contains(PanelRect r, int x, int y) noexcept
```

True when (x, y) lies inside `r`, edges included.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:152`](../../include/shulib/calibration/calibrate_panel.hpp#L152).*

<a id="overlaps"></a>

## `overlaps`

```cpp
[[nodiscard]] constexpr bool overlaps(PanelRect a, PanelRect b) noexcept
```

True when `a` and `b` share at least one pixel.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:157`](../../include/shulib/calibration/calibrate_panel.hpp#L157).*

<a id="within"></a>

## `within`

```cpp
[[nodiscard]] constexpr bool within(PanelRect inner, PanelRect outer) noexcept
```

True when `inner` lies wholly inside `outer`.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:162`](../../include/shulib/calibration/calibrate_panel.hpp#L162).*

<a id="kheaderbar"></a>

## `kHeaderBar`

```cpp
inline constexpr PanelRect kHeaderBar{0, 0, 479, 26}
```

The header bar: its colour is the state (blue unpowered, amber powered, green saved, red stopped).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:169`](../../include/shulib/calibration/calibrate_panel.hpp#L169).*

<a id="kprimaryslot"></a>

## `kPrimarySlot`

```cpp
inline constexpr PanelRect kPrimarySlot{6, 180, 300, 234}
```

DONE's place, the primary slot at the bottom left.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:171`](../../include/shulib/calibration/calibrate_panel.hpp#L171).*

<a id="kabortslot"></a>

## `kAbortSlot`

```cpp
inline constexpr PanelRect kAbortSlot{330, 180, 474, 234}
```

ABORT's place in EVERY state that shows it — the bottom right. Never moves under a thumb.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:173`](../../include/shulib/calibration/calibrate_panel.hpp#L173).*

<a id="kabortrow"></a>

## `kAbortRow`

```cpp
inline constexpr PanelRect kAbortRow{6, 180, 474, 234}
```

ABORT in the two POWERED phases: the whole button row, still containing kAbortSlot — the PULSES' only once their first pulse has driven a motor (until then kAbortSlot: DONE's touch leads there, and kPrimarySlot lies in this row), the SPIN's throughout.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:177`](../../include/shulib/calibration/calibrate_panel.hpp#L177).*

<a id="kchoiceleftbox"></a>

## `kChoiceLeftBox`

```cpp
inline constexpr PanelRect kChoiceLeftBox{6, 72, 234, 158}
```

A held screen's left box (USE NEW / SAVE, CAL1): a big box in the MIDDLE, never in the button row — the rectangle FRONT FIRST had until CAL1 Part B removed the WHICH END question.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:180`](../../include/shulib/calibration/calibrate_panel.hpp#L180).*

<a id="kchoicerightbox"></a>

## `kChoiceRightBox`

```cpp
inline constexpr PanelRect kChoiceRightBox{246, 72, 474, 158}
```

A held screen's RUN AGAIN: the same, on the right (BACK FIRST's rectangle until CAL1 Part B).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:182`](../../include/shulib/calibration/calibrate_panel.hpp#L182).*

<a id="krunagainbox"></a>

## `kRunAgainBox`

```cpp
inline constexpr PanelRect kRunAgainBox{246, 112, 474, 158}
```

RUN AGAIN on the outcome screens: middle right, clear of the button row.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:184`](../../include/shulib/calibration/calibrate_panel.hpp#L184).*

<a id="enum-class-textsize"></a>

## `enum class TextSize`

```cpp
enum class TextSize : std::uint8_t
```

The three text sizes PROS offers.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:187`](../../include/shulib/calibration/calibrate_panel.hpp#L187).*

<a id="textsize-small"></a>

### `TextSize::Small`

```cpp
Small
```

kSmallCols per line

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:188`](../../include/shulib/calibration/calibrate_panel.hpp#L188).*

<a id="textsize-medium"></a>

### `TextSize::Medium`

```cpp
Medium
```

kMediumCols per line

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:189`](../../include/shulib/calibration/calibrate_panel.hpp#L189).*

<a id="textsize-large"></a>

### `TextSize::Large`

```cpp
Large
```

kLargeCols per line

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:190`](../../include/shulib/calibration/calibrate_panel.hpp#L190).*

<a id="budgetfor"></a>

## `budgetFor`

```cpp
[[nodiscard]] constexpr std::size_t budgetFor(TextSize size) noexcept
```

The budget, in characters, of one line of `size`.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:194`](../../include/shulib/calibration/calibrate_panel.hpp#L194).*

<a id="heightfor"></a>

## `heightFor`

```cpp
[[nodiscard]] constexpr std::int16_t heightFor(TextSize size) noexcept
```

The height a line of `size` is laid out for, pixels (an ESTIMATE).

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:199`](../../include/shulib/calibration/calibrate_panel.hpp#L199).*

<a id="enum-class-paneltone"></a>

## `enum class PanelTone`

```cpp
enum class PanelTone : std::uint8_t
```

A colour by MEANING; the program maps each to RGB.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:206`](../../include/shulib/calibration/calibrate_panel.hpp#L206).*

<a id="paneltone-blue"></a>

### `PanelTone::Blue`

```cpp
Blue
```

waiting / unpowered (the header)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:207`](../../include/shulib/calibration/calibrate_panel.hpp#L207).*

<a id="paneltone-amber"></a>

### `PanelTone::Amber`

```cpp
Amber
```

powered — stand clear; or "not yet" on a number

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:208`](../../include/shulib/calibration/calibrate_panel.hpp#L208).*

<a id="paneltone-green"></a>

### `PanelTone::Green`

```cpp
Green
```

ready / saved / past the floor

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:209`](../../include/shulib/calibration/calibrate_panel.hpp#L209).*

<a id="paneltone-red"></a>

### `PanelTone::Red`

```cpp
Red
```

stopped / not saved / out of bounds

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:210`](../../include/shulib/calibration/calibrate_panel.hpp#L210).*

<a id="paneltone-grey"></a>

### `PanelTone::Grey`

```cpp
Grey
```

a greyed button, or a quiet line

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:211`](../../include/shulib/calibration/calibrate_panel.hpp#L211).*

<a id="paneltone-white"></a>

### `PanelTone::White`

```cpp
White
```

ordinary text

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:212`](../../include/shulib/calibration/calibrate_panel.hpp#L212).*

<a id="enum-class-panelscreen"></a>

## `enum class PanelScreen`

```cpp
enum class PanelScreen : std::uint8_t
```

What a screen is showing.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:216`](../../include/shulib/calibration/calibrate_panel.hpp#L216).*

<a id="panelscreen-notstarted"></a>

### `PanelScreen::NotStarted`

```cpp
NotStarted
```

the calibrator is Idle

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:217`](../../include/shulib/calibration/calibrate_panel.hpp#L217).*

<a id="panelscreen-imuwait"></a>

### `PanelScreen::ImuWait`

```cpp
ImuWait
```

hands off while the IMU starts

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:218`](../../include/shulib/calibration/calibrate_panel.hpp#L218).*

<a id="panelscreen-push"></a>

### `PanelScreen::Push`

```cpp
Push
```

push it, FRONT END FIRST, along the tape

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:219`](../../include/shulib/calibration/calibrate_panel.hpp#L219).*

<a id="panelscreen-pulses"></a>

### `PanelScreen::Pulses`

```cpp
Pulses
```

powered: one motor at a time (AMBER)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:220`](../../include/shulib/calibration/calibrate_panel.hpp#L220).*

<a id="panelscreen-spin"></a>

### `PanelScreen::Spin`

```cpp
Spin
```

powered: the spin (AMBER)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:221`](../../include/shulib/calibration/calibrate_panel.hpp#L221).*

<a id="panelscreen-saved"></a>

### `PanelScreen::Saved`

```cpp
Saved
```

the run finished (SAVED / NOT SAVED / MEASURED)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:222`](../../include/shulib/calibration/calibrate_panel.hpp#L222).*

<a id="panelscreen-stopped"></a>

### `PanelScreen::Stopped`

```cpp
Stopped
```

the run was refused or aborted

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:223`](../../include/shulib/calibration/calibrate_panel.hpp#L223).*

<a id="panelscreen-held"></a>

### `PanelScreen::Held`

```cpp
Held
```

(CAL1) the save WAITS for the operator: SAVE (or USE NEW) or RUN AGAIN

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:224`](../../include/shulib/calibration/calibrate_panel.hpp#L224).*

<a id="enum-class-saveoutcome"></a>

## `enum class SaveOutcome`

```cpp
enum class SaveOutcome : std::uint8_t
```

Whether the program managed to write the result — the panel cannot know, so it is told.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:228`](../../include/shulib/calibration/calibrate_panel.hpp#L228).*

<a id="saveoutcome-notattempted"></a>

### `SaveOutcome::NotAttempted`

```cpp
NotAttempted
```

not written (yet): the header says MEASURED

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:229`](../../include/shulib/calibration/calibrate_panel.hpp#L229).*

<a id="saveoutcome-saved"></a>

### `SaveOutcome::Saved`

```cpp
Saved
```

written, flushed, read back and compared: SAVED

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:230`](../../include/shulib/calibration/calibrate_panel.hpp#L230).*

<a id="saveoutcome-notsaved"></a>

### `SaveOutcome::NotSaved`

```cpp
NotSaved
```

the write or the read-back failed: NOT SAVED, copy the numbers

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:231`](../../include/shulib/calibration/calibrate_panel.hpp#L231).*

<a id="saveoutcome-held"></a>

### `SaveOutcome::Held`

```cpp
Held
```

(CAL1) not written YET: the operator chooses (HoldReason says why)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:232`](../../include/shulib/calibration/calibrate_panel.hpp#L232).*

<a id="enum-class-frontchoice"></a>

## `enum class FrontChoice`

```cpp
enum class FrontChoice : std::uint8_t
```

Which front a SAVED record carries when the operator was asked (CAL1) — the header says it.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:236`](../../include/shulib/calibration/calibrate_panel.hpp#L236).*

<a id="frontchoice-none"></a>

### `FrontChoice::None`

```cpp
None
```

not asked: no FRONT FLIPPED hold — the same front, the card's first record, or a front that could not be checked (files that do not decode, or CANNOT TELL)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:239`](../../include/shulib/calibration/calibrate_panel.hpp#L239).*

<a id="frontchoice-new"></a>

### `FrontChoice::New`

```cpp
New
```

USE NEW: the end pushed first this time is the front, against the card's record

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:240`](../../include/shulib/calibration/calibrate_panel.hpp#L240).*

<a id="enum-class-holdreason"></a>

## `enum class HoldReason`

```cpp
enum class HoldReason : std::uint8_t
```

Why a save is HELD (CAL1): the screen the operator answers. One question per screen, in this order: FRONT FLIPPED, then MECHANISM PORT TURNED — one screen per declared port that turned (FinishFacts::holdPort).

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:246`](../../include/shulib/calibration/calibrate_panel.hpp#L246).*

<a id="holdreason-none"></a>

### `HoldReason::None`

```cpp
None
```

not held

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:247`](../../include/shulib/calibration/calibrate_panel.hpp#L247).*

<a id="holdreason-frontflipped"></a>

### `HoldReason::FrontFlipped`

```cpp
FrontFlipped
```

the card's record calls the end that led this push its BACK

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:248`](../../include/shulib/calibration/calibrate_panel.hpp#L248).*

<a id="holdreason-mechanismturned"></a>

### `HoldReason::MechanismTurned`

```cpp
MechanismTurned
```

a DECLARED mechanism motor turned with the push, as a drive motor would

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:249`](../../include/shulib/calibration/calibrate_panel.hpp#L249).*

<a id="struct-finishfacts"></a>

## `struct FinishFacts`

```cpp
struct FinishFacts
```

What the program tells the finished and held screens (CAL1): the panel cannot know these. The default is the plain finished screen — no hold, no NOTE, no front choice.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:254`](../../include/shulib/calibration/calibrate_panel.hpp#L254).*

<a id="finishfacts-front"></a>

### `FinishFacts::front`

```cpp
FrontChoice front = FrontChoice::None
```

New: the SAVED header says the front is new

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:255`](../../include/shulib/calibration/calibrate_panel.hpp#L255).*

<a id="finishfacts-hold"></a>

### `FinishFacts::hold`

```cpp
HoldReason hold = HoldReason::None
```

which held screen — read only with SaveOutcome::Held

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:256`](../../include/shulib/calibration/calibrate_panel.hpp#L256).*

<a id="finishfacts-holdport"></a>

### `FinishFacts::holdPort`

```cpp
int holdPort = 0
```

MechanismTurned: the declared port this screen asks about — one screen per port that turned. A different port is a NEW question: TouchGate::view() restarts the arming clock (re-greys the boxes), as it does when the reason changes. 0 = none.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:260`](../../include/shulib/calibration/calibrate_panel.hpp#L260).*

<a id="finishfacts-holdline1"></a>

### `FinishFacts::holdLine1`

```cpp
const char* holdLine1 = nullptr
```

MechanismTurned, MEDIUM: "port 6: 1692 deg, drive 1690 deg"

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:261`](../../include/shulib/calibration/calibrate_panel.hpp#L261).*

<a id="finishfacts-holdline2"></a>

### `FinishFacts::holdLine2`

```cpp
const char* holdLine2 = nullptr
```

MechanismTurned, SMALL: what the reference — the card's record, or (D6 (a)) the typed drive ports — says of the port: "the card's record DRIVES with port 12", or null

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:264`](../../include/shulib/calibration/calibrate_panel.hpp#L264).*

<a id="finishfacts-note"></a>

### `FinishFacts::note`

```cpp
const char* note = nullptr
```

the finished screen's one amber NOTE row, or null

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:265`](../../include/shulib/calibration/calibrate_panel.hpp#L265).*

<a id="finishfacts-typedports"></a>

### `FinishFacts::typedPorts`

```cpp
bool typedPorts = false
```

FrontFlipped: the front was checked against the team's TYPED drive ports (the card held no usable record — CAL1 D6 (a)), so the screen paints their words, never the card's.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:268`](../../include/shulib/calibration/calibrate_panel.hpp#L268).*

<a id="enum-class-panelbuttonid"></a>

## `enum class PanelButtonId`

```cpp
enum class PanelButtonId : std::uint8_t
```

Which button.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:272`](../../include/shulib/calibration/calibrate_panel.hpp#L272).*

<a id="panelbuttonid-done"></a>

### `PanelButtonId::Done`

```cpp
Done
```

"I have finished pushing" — drawn only when pushReady()

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:273`](../../include/shulib/calibration/calibrate_panel.hpp#L273).*

<a id="panelbuttonid-saveheld"></a>

### `PanelButtonId::SaveHeld`

```cpp
SaveHeld
```

(CAL1) a held screen's left box: USE NEW / SAVE — save this run as measured

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:274`](../../include/shulib/calibration/calibrate_panel.hpp#L274).*

<a id="panelbuttonid-abort"></a>

### `PanelButtonId::Abort`

```cpp
Abort
```

stop now — always armed

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:275`](../../include/shulib/calibration/calibrate_panel.hpp#L275).*

<a id="panelbuttonid-runagain"></a>

### `PanelButtonId::RunAgain`

```cpp
RunAgain
```

on the outcome screens: the program rebuilds the calibrator

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:276`](../../include/shulib/calibration/calibrate_panel.hpp#L276).*

<a id="struct-panelbutton"></a>

## `struct PanelButton`

```cpp
struct PanelButton
```

One button of a view.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:280`](../../include/shulib/calibration/calibrate_panel.hpp#L280).*

<a id="panelbutton-id"></a>

### `PanelButton::id`

```cpp
PanelButtonId id = PanelButtonId::Abort
```

Which button.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:282`](../../include/shulib/calibration/calibrate_panel.hpp#L282).*

<a id="panelbutton-rect"></a>

### `PanelButton::rect`

```cpp
PanelRect rect{}
```

Where it is drawn (its hit box is hitBox(rect)).

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:284`](../../include/shulib/calibration/calibrate_panel.hpp#L284).*

<a id="panelbutton-label"></a>

### `PanelButton::label`

```cpp
const char* label = ""
```

Its label, MEDIUM, ASCII.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:286`](../../include/shulib/calibration/calibrate_panel.hpp#L286).*

<a id="panelbutton-armed"></a>

### `PanelButton::armed`

```cpp
bool armed = true
```

False while the arming delay runs: drawn GREYED and never hit.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:288`](../../include/shulib/calibration/calibrate_panel.hpp#L288).*

<a id="kpanellinecap"></a>

## `kPanelLineCap`

```cpp
inline constexpr std::size_t kPanelLineCap = kSmallCols + 1
```

The longest line a view carries, plus its terminator.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:292`](../../include/shulib/calibration/calibrate_panel.hpp#L292).*

<a id="kpanelmaxlines"></a>

## `kPanelMaxLines`

```cpp
inline constexpr std::size_t kPanelMaxLines = 10
```

The most text lines one view carries.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:294`](../../include/shulib/calibration/calibrate_panel.hpp#L294).*

<a id="kpanelmaxbuttons"></a>

## `kPanelMaxButtons`

```cpp
inline constexpr std::size_t kPanelMaxButtons = 3
```

The most buttons one view carries.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:296`](../../include/shulib/calibration/calibrate_panel.hpp#L296).*

<a id="struct-panelline"></a>

## `struct PanelLine`

```cpp
struct PanelLine
```

One line of text at a fixed place and size.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:299`](../../include/shulib/calibration/calibrate_panel.hpp#L299).*

<a id="panelline-x"></a>

### `PanelLine::x`

```cpp
std::int16_t x = 0
```

Left edge, pixels.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:301`](../../include/shulib/calibration/calibrate_panel.hpp#L301).*

<a id="panelline-y"></a>

### `PanelLine::y`

```cpp
std::int16_t y = 0
```

Top edge, pixels.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:303`](../../include/shulib/calibration/calibrate_panel.hpp#L303).*

<a id="panelline-size"></a>

### `PanelLine::size`

```cpp
TextSize size = TextSize::Medium
```

Its size; the text fits budgetFor(size) (the host tests check every reachable line).

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:305`](../../include/shulib/calibration/calibrate_panel.hpp#L305).*

<a id="panelline-tone"></a>

### `PanelLine::tone`

```cpp
PanelTone tone = PanelTone::White
```

Its colour, by meaning.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:307`](../../include/shulib/calibration/calibrate_panel.hpp#L307).*

<a id="panelline-text"></a>

### `PanelLine::text`

```cpp
char text[kPanelLineCap]{}
```

The text, NUL-terminated, ASCII, no '%'.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:309`](../../include/shulib/calibration/calibrate_panel.hpp#L309).*

<a id="struct-panelview"></a>

## `struct PanelView`

```cpp
struct PanelView
```

Everything one frame of the screen says, and which touches mean something.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:313`](../../include/shulib/calibration/calibrate_panel.hpp#L313).*

<a id="panelview-screen"></a>

### `PanelView::screen`

```cpp
PanelScreen screen = PanelScreen::NotStarted
```

What the screen is showing.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:315`](../../include/shulib/calibration/calibrate_panel.hpp#L315).*

<a id="panelview-headertone"></a>

### `PanelView::headerTone`

```cpp
PanelTone headerTone = PanelTone::Blue
```

The header bar's colour — the state at a glance.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:317`](../../include/shulib/calibration/calibrate_panel.hpp#L317).*

<a id="panelview-headerword"></a>

### `PanelView::headerWord`

```cpp
const char* headerWord = ""
```

The step word, left of the header bar, MEDIUM, at most kHeaderCols.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:319`](../../include/shulib/calibration/calibrate_panel.hpp#L319).*

<a id="panelview-lines"></a>

### `PanelView::lines`

```cpp
PanelLine lines[kPanelMaxLines]{}
```

The text lines, in drawing order.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:321`](../../include/shulib/calibration/calibrate_panel.hpp#L321).*

<a id="panelview-linecount"></a>

### `PanelView::lineCount`

```cpp
std::uint8_t lineCount = 0
```

How many entries of `lines` are real.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:323`](../../include/shulib/calibration/calibrate_panel.hpp#L323).*

<a id="panelview-buttons"></a>

### `PanelView::buttons`

```cpp
PanelButton buttons[kPanelMaxButtons]{}
```

The buttons (live or greyed), ABORT always last when present.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:325`](../../include/shulib/calibration/calibrate_panel.hpp#L325).*

<a id="panelview-buttoncount"></a>

### `PanelView::buttonCount`

```cpp
std::uint8_t buttonCount = 0
```

How many entries of `buttons` are real.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:327`](../../include/shulib/calibration/calibrate_panel.hpp#L327).*

<a id="struct-stoppedwords"></a>

## `struct StoppedWords`

```cpp
struct StoppedWords
```

The STOPPED screen's words for one refusal code.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:331`](../../include/shulib/calibration/calibrate_panel.hpp#L331).*

<a id="stoppedwords-headline"></a>

### `StoppedWords::headline`

```cpp
const char* headline
```

LARGE, at most kLargeCols.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:333`](../../include/shulib/calibration/calibrate_panel.hpp#L333).*

<a id="stoppedwords-why"></a>

### `StoppedWords::why`

```cpp
const char* why
```

MEDIUM, at most kMediumCols: what happened.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:335`](../../include/shulib/calibration/calibrate_panel.hpp#L335).*

<a id="stoppedwords-doline"></a>

### `StoppedWords::doLine`

```cpp
const char* doLine
```

MEDIUM, at most kMediumCols: what to do.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:337`](../../include/shulib/calibration/calibrate_panel.hpp#L337).*

<a id="stoppedwords"></a>

## `stoppedWords`

```cpp
[[nodiscard]] constexpr StoppedWords stoppedWords(Refusal code) noexcept
```

The STOPPED wording for `code` — keyed on the CODE, never on the reason's prose, which the screen shows below it in SMALL for the numbers. Every entry is checked at compile time.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:342`](../../include/shulib/calibration/calibrate_panel.hpp#L342).*

<a id="screentext"></a>

## `screenText`

```cpp
[[nodiscard]] constexpr bool screenText(std::string_view s, std::size_t cols) noexcept
```

True when `s` may be drawn in a line of `cols` characters: printable ASCII only, no '%', and no longer than `cols`. The compile-time gate for every literal below.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:395`](../../include/shulib/calibration/calibrate_panel.hpp#L395).*

<a id="kheaderidle"></a>

## `kHeaderIdle`

```cpp
inline constexpr std::string_view kHeaderIdle = "CALIBRATE"
```

Header words (kHeaderCols).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:412`](../../include/shulib/calibration/calibrate_panel.hpp#L412).*

<a id="kheaderimu"></a>

## `kHeaderImu`

```cpp
inline constexpr std::string_view kHeaderImu = "1/4 IMU"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:414`](../../include/shulib/calibration/calibrate_panel.hpp#L414).*

<a id="kheaderpush"></a>

## `kHeaderPush`

```cpp
inline constexpr std::string_view kHeaderPush = "2/4 PUSH"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:416`](../../include/shulib/calibration/calibrate_panel.hpp#L416).*

<a id="kheaderpulses"></a>

## `kHeaderPulses`

```cpp
inline constexpr std::string_view kHeaderPulses = "3/4 PULSES"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:418`](../../include/shulib/calibration/calibrate_panel.hpp#L418).*

<a id="kheaderspin"></a>

## `kHeaderSpin`

```cpp
inline constexpr std::string_view kHeaderSpin = "4/4 SPIN"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:420`](../../include/shulib/calibration/calibrate_panel.hpp#L420).*

<a id="kheadersaved"></a>

## `kHeaderSaved`

```cpp
inline constexpr std::string_view kHeaderSaved = "SAVED"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:422`](../../include/shulib/calibration/calibrate_panel.hpp#L422).*

<a id="kheadernotsaved"></a>

## `kHeaderNotSaved`

```cpp
inline constexpr std::string_view kHeaderNotSaved = "NOT SAVED"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:424`](../../include/shulib/calibration/calibrate_panel.hpp#L424).*

<a id="kheadermeasured"></a>

## `kHeaderMeasured`

```cpp
inline constexpr std::string_view kHeaderMeasured = "MEASURED"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:426`](../../include/shulib/calibration/calibrate_panel.hpp#L426).*

<a id="kheaderstopped"></a>

## `kHeaderStopped`

```cpp
inline constexpr std::string_view kHeaderStopped = "STOPPED -- MOTORS OFF"
```

See kHeaderIdle.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:428`](../../include/shulib/calibration/calibrate_panel.hpp#L428).*

<a id="knotstarted"></a>

## `kNotStarted`

```cpp
inline constexpr std::string_view kNotStarted = "NOT STARTED"
```

Instructions (kLargeCols).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:430`](../../include/shulib/calibration/calibrate_panel.hpp#L430).*

<a id="khandsoff"></a>

## `kHandsOff`

```cpp
inline constexpr std::string_view kHandsOff = "HANDS OFF"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:432`](../../include/shulib/calibration/calibrate_panel.hpp#L432).*

<a id="kimuwarming"></a>

## `kImuWarming`

```cpp
inline constexpr std::string_view kImuWarming = "IMU WARMING UP"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:434`](../../include/shulib/calibration/calibrate_panel.hpp#L434).*

<a id="kstandclear"></a>

## `kStandClear`

```cpp
inline constexpr std::string_view kStandClear = "STAND CLEAR"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:436`](../../include/shulib/calibration/calibrate_panel.hpp#L436).*

<a id="ktwitching"></a>

## `kTwitching`

```cpp
inline constexpr std::string_view kTwitching = "TWITCHING MOTORS"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:438`](../../include/shulib/calibration/calibrate_panel.hpp#L438).*

<a id="kspinning"></a>

## `kSpinning`

```cpp
inline constexpr std::string_view kSpinning = "SPINNING"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:440`](../../include/shulib/calibration/calibrate_panel.hpp#L440).*

<a id="ksettling"></a>

## `kSettling`

```cpp
inline constexpr std::string_view kSettling = "SETTLING"
```

See kNotStarted.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:442`](../../include/shulib/calibration/calibrate_panel.hpp#L442).*

<a id="kfindingleft"></a>

## `kFindingLeft`

```cpp
inline constexpr std::string_view kFindingLeft = "FINDING LEFT"
```

See kNotStarted (CAL2: the turn that names the groups' sides).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:444`](../../include/shulib/calibration/calibrate_panel.hpp#L444).*

<a id="knextdonewhenmoved"></a>

## `kNextDoneWhenMoved`

```cpp
inline constexpr std::string_view kNextDoneWhenMoved = "DONE shows once the motors turn"
```

NEXT / detail lines (kMediumCols).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:446`](../../include/shulib/calibration/calibrate_panel.hpp#L446).*

<a id="knextletgo"></a>

## `kNextLetGo`

```cpp
inline constexpr std::string_view kNextLetGo = "let go -- DONE shows when it stops"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:448`](../../include/shulib/calibration/calibrate_panel.hpp#L448).*

<a id="knexttouchdone"></a>

## `kNextTouchDone`

```cpp
inline constexpr std::string_view kNextTouchDone = "touch DONE -- then stand clear"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:450`](../../include/shulib/calibration/calibrate_panel.hpp#L450).*

<a id="knexttwitch"></a>

## `kNextTwitch`

```cpp
inline constexpr std::string_view kNextTwitch = "NEXT: motors twitch -- stand clear"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:452`](../../include/shulib/calibration/calibrate_panel.hpp#L452).*

<a id="knextspins"></a>

## `kNextSpins`

```cpp
inline constexpr std::string_view kNextSpins = "NEXT: it spins -- stand clear"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:454`](../../include/shulib/calibration/calibrate_panel.hpp#L454).*

<a id="knextsaves"></a>

## `kNextSaves`

```cpp
inline constexpr std::string_view kNextSaves = "NEXT: it saves -- stay clear"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:456`](../../include/shulib/calibration/calibrate_panel.hpp#L456).*

<a id="ksettlinghandsoff"></a>

## `kSettlingHandsOff`

```cpp
inline constexpr std::string_view kSettlingHandsOff = "settling -- hands off"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:458`](../../include/shulib/calibration/calibrate_panel.hpp#L458).*

<a id="kspinagain"></a>

## `kSpinAgain`

```cpp
inline constexpr std::string_view kSpinAgain = "hands off -- it spins again"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:460`](../../include/shulib/calibration/calibrate_panel.hpp#L460).*

<a id="kspinfirst"></a>

## `kSpinFirst`

```cpp
inline constexpr std::string_view kSpinFirst = "hands off -- it spins next"
```

See kNextDoneWhenMoved.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:462`](../../include/shulib/calibration/calibrate_panel.hpp#L462).*

<a id="kcopythenrunagain"></a>

## `kCopyThenRunAgain`

```cpp
inline constexpr std::string_view kCopyThenRunAgain = "NEXT: copy these, then RUN AGAIN to check"
```

Outcome lines (kSmallCols).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:464`](../../include/shulib/calibration/calibrate_panel.hpp#L464).*

<a id="kcopynow"></a>

## `kCopyNow`

```cpp
inline constexpr std::string_view kCopyNow = "NOT SAVED -- copy these numbers NOW"
```

See kCopyThenRunAgain.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:466`](../../include/shulib/calibration/calibrate_panel.hpp#L466).*

<a id="kcopythese"></a>

## `kCopyThese`

```cpp
inline constexpr std::string_view kCopyThese = "copy these numbers"
```

See kCopyThenRunAgain.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:468`](../../include/shulib/calibration/calibrate_panel.hpp#L468).*

<a id="kfullreasononserial"></a>

## `kFullReasonOnSerial`

```cpp
inline constexpr std::string_view kFullReasonOnSerial = "the full reason is on the serial log"
```

See kCopyThenRunAgain.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:470`](../../include/shulib/calibration/calibrate_panel.hpp#L470).*

<a id="klabeldone"></a>

## `kLabelDone`

```cpp
inline constexpr std::string_view kLabelDone = "DONE"
```

Button labels (MEDIUM; a label also fits its box: 12 characters).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:472`](../../include/shulib/calibration/calibrate_panel.hpp#L472).*

<a id="klabelabort"></a>

## `kLabelAbort`

```cpp
inline constexpr std::string_view kLabelAbort = "ABORT"
```

See kLabelDone.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:474`](../../include/shulib/calibration/calibrate_panel.hpp#L474).*

<a id="klabelrunagain"></a>

## `kLabelRunAgain`

```cpp
inline constexpr std::string_view kLabelRunAgain = "RUN AGAIN"
```

See kLabelDone.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:476`](../../include/shulib/calibration/calibrate_panel.hpp#L476).*

<a id="kheaderflipped"></a>

## `kHeaderFlipped`

```cpp
inline constexpr std::string_view kHeaderFlipped = "FRONT FLIPPED"
```

Header word: the FRONT FLIPPED held screen (CAL1).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:478`](../../include/shulib/calibration/calibrate_panel.hpp#L478).*

<a id="kheadermechturned"></a>

## `kHeaderMechTurned`

```cpp
inline constexpr std::string_view kHeaderMechTurned = "MECHANISM PORT TURNED"
```

Header word: the held screen when a declared mechanism motor turned with the push (CAL1).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:480`](../../include/shulib/calibration/calibrate_panel.hpp#L480).*

<a id="kheadersavednew"></a>

## `kHeaderSavedNew`

```cpp
inline constexpr std::string_view kHeaderSavedNew = "SAVED -- NEW FRONT"
```

Header word: SAVED after USE NEW (CAL1).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:482`](../../include/shulib/calibration/calibrate_panel.hpp#L482).*

<a id="kpushfrontfirst"></a>

## `kPushFrontFirst`

```cpp
inline constexpr std::string_view kPushFrontFirst = "PUSH FRONT FIRST"
```

LARGE: the push instruction — the end that leads the push is the FRONT (CAL1).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:484`](../../include/shulib/calibration/calibrate_panel.hpp#L484).*

<a id="knothingsavedyet"></a>

## `kNothingSavedYet`

```cpp
inline constexpr std::string_view kNothingSavedYet = "NOTHING SAVED YET"
```

LARGE: every held screen's headline.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:486`](../../include/shulib/calibration/calibrate_panel.hpp#L486).*

<a id="kfrontrule"></a>

## `kFrontRule`

```cpp
inline constexpr std::string_view kFrontRule = "the end that leads the push is FRONT"
```

MEDIUM: the rule, shown while the IMU warms up (CAL1).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:488`](../../include/shulib/calibration/calibrate_panel.hpp#L488).*

<a id="kflipwhat"></a>

## `kFlipWhat`

```cpp
inline constexpr std::string_view kFlipWhat = "the card calls the end you led BACK"
```

MEDIUM: FRONT FLIPPED — what the records say (never what the operator did: a LEFT/RIGHT cable swap gives the same record as the other end leading).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:491`](../../include/shulib/calibration/calibrate_panel.hpp#L491).*

<a id="kflipagain"></a>

## `kFlipAgain`

```cpp
inline constexpr std::string_view kFlipAgain = "RUN AGAIN: you led with the BACK by mistake"
```

SMALL: FRONT FLIPPED — RUN AGAIN's case.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:493`](../../include/shulib/calibration/calibrate_panel.hpp#L493).*

<a id="kflipnew"></a>

## `kFlipNew`

```cpp
inline constexpr std::string_view kFlipNew = "USE NEW: you led with the FRONT (cables swapped?)"
```

SMALL: FRONT FLIPPED — USE NEW's case (a cable swap between the sides looks exactly like a flip).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:495`](../../include/shulib/calibration/calibrate_panel.hpp#L495).*

<a id="kflipwarn"></a>

## `kFlipWarn`

```cpp
inline constexpr std::string_view kFlipWarn = "USE NEW after a wrong push reverses every auton"
```

SMALL: FRONT FLIPPED — the warning.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:497`](../../include/shulib/calibration/calibrate_panel.hpp#L497).*

<a id="kmechnotoncard"></a>

## `kMechNotOnCard`

```cpp
inline constexpr std::string_view kMechNotOnCard = "it is declared a mechanism: not in this record"
```

SMALL: MECHANISM PORT TURNED — when the card's record does not drive with the port.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:499`](../../include/shulib/calibration/calibrate_panel.hpp#L499).*

<a id="kmechsave"></a>

## `kMechSave`

```cpp
inline constexpr std::string_view kMechSave = "SAVE: it is a mechanism the push turns -- keep it"
```

SMALL: MECHANISM PORT TURNED — SAVE's case.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:501`](../../include/shulib/calibration/calibrate_panel.hpp#L501).*

<a id="kmechagain"></a>

## `kMechAgain`

```cpp
inline constexpr std::string_view kMechAgain = "RUN AGAIN = save nothing: a drive motor? fix facts"
```

SMALL: MECHANISM PORT TURNED — RUN AGAIN's case.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:503`](../../include/shulib/calibration/calibrate_panel.hpp#L503).*

<a id="klabelnew"></a>

## `kLabelNew`

```cpp
inline constexpr std::string_view kLabelNew = "USE NEW"
```

Label: the held screen's left box (PanelButtonId::SaveHeld) on FRONT FLIPPED.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:505`](../../include/shulib/calibration/calibrate_panel.hpp#L505).*

<a id="klabelsave"></a>

## `kLabelSave`

```cpp
inline constexpr std::string_view kLabelSave = "SAVE"
```

Label: the held screen's left box on MECHANISM PORT TURNED.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:507`](../../include/shulib/calibration/calibrate_panel.hpp#L507).*

<a id="knotefirst"></a>

## `kNoteFirst`

```cpp
inline constexpr std::string_view kNoteFirst = "FIRST RECORD -- the end you pushed first is FRONT"
```

SMALL: the finished screen's NOTE after a first record (no record on the card).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:509`](../../include/shulib/calibration/calibrate_panel.hpp#L509).*

<a id="knotenotchecked"></a>

## `kNoteNotChecked`

```cpp
inline constexpr std::string_view kNoteNotChecked = "FRONT NOT CHECKED -- see the log"
```

SMALL: the NOTE when the front could not be checked (the card's record does not decode, or the check could not tell).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:512`](../../include/shulib/calibration/calibrate_panel.hpp#L512).*

<a id="kwarningcut"></a>

## `kWarningCut`

```cpp
inline constexpr std::string_view kWarningCut = " ..."
```

SMALL: the warning row's ending when the NOTE takes its second line (CAL1).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:514`](../../include/shulib/calibration/calibrate_panel.hpp#L514).*

<a id="kflipwhattyped"></a>

## `kFlipWhatTyped`

```cpp
inline constexpr std::string_view kFlipWhatTyped = "typed ports: the end you led is BACK"
```

MEDIUM: FRONT FLIPPED against the team's TYPED drive ports (CAL1 D6 (a)) — what they say.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:516`](../../include/shulib/calibration/calibrate_panel.hpp#L516).*

<a id="kflipnewtyped"></a>

## `kFlipNewTyped`

```cpp
inline constexpr std::string_view kFlipNewTyped = "USE NEW: you led with the FRONT -- fix typed ports"
```

SMALL: FRONT FLIPPED against the typed ports — USE NEW's case: the FRONT led, so the typed ports (or the cables since they were typed) are wrong.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:519`](../../include/shulib/calibration/calibrate_panel.hpp#L519).*

<a id="kflipwarntyped"></a>

## `kFlipWarnTyped`

```cpp
inline constexpr std::string_view kFlipWarnTyped = "USE NEW reverses autons tuned on the typed ports"
```

SMALL: FRONT FLIPPED against the typed ports — the warning: the autons tuned on them.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:522`](../../include/shulib/calibration/calibrate_panel.hpp#L522).*

<a id="knotetypedsame"></a>

## `kNoteTypedSame`

```cpp
inline constexpr std::string_view kNoteTypedSame = "FRONT MATCHES YOUR TYPED DRIVE PORTS"
```

SMALL: the NOTE when the card held no usable record and EVERY port of the typed drive ports and this push agrees (same side, same sign, none in one list only).

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:526`](../../include/shulib/calibration/calibrate_panel.hpp#L526).*

<a id="knotefixtyped"></a>

## `kNoteFixTyped`

```cpp
inline constexpr std::string_view kNoteFixTyped = "FIX YOUR TYPED DRIVE PORTS -- see the log"
```

SMALL: the NOTE after USE NEW against the typed drive ports: they call the saved front the back.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:528`](../../include/shulib/calibration/calibrate_panel.hpp#L528).*

<a id="knotetypedpartial"></a>

## `kNoteTypedPartial`

```cpp
inline constexpr std::string_view kNoteTypedPartial = "FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log"
```

SMALL: the NOTE when the front matched the typed drive ports but some port did not (a sign, a side, a port in one list only): the serial log names each one.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:531`](../../include/shulib/calibration/calibrate_panel.hpp#L531).*

<a id="struct-checked"></a>

## `struct Checked`

```cpp
struct Checked
```

One literal and the budget it must fit.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:535`](../../include/shulib/calibration/calibrate_panel.hpp#L535).*

<a id="checked-text"></a>

### `Checked::text`

```cpp
std::string_view text
```

the literal

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:536`](../../include/shulib/calibration/calibrate_panel.hpp#L536).*

<a id="checked-cols"></a>

### `Checked::cols`

```cpp
std::size_t cols
```

its line's budget

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:537`](../../include/shulib/calibration/calibrate_panel.hpp#L537).*

<a id="kall"></a>

## `kAll`

```cpp
inline constexpr Checked kAll[] = { {kHeaderIdle, kHeaderCols}, {kHeaderImu, kHeaderCols}, {kHeaderPush, kHeaderCols}, {kHeaderPulses, kHeaderCols}, {kHeaderSpin, kHeaderCols}, {kHeaderSaved, kHeaderCols}, {kHeaderNotSaved, kHeaderCols}, {kHeaderMeasured, kHeaderCols}, {kHeaderStopped, kHeaderCols}, {kNotStarted, kLargeCols}, {kHandsOff, kLargeCols}, {kImuWarming, kLargeCols}, {kPushFrontFirst, kLargeCols}, {kStandClear, kLargeCols}, {kTwitching, kLargeCols}, {kSpinning, kLargeCols}, {kSettling, kLargeCols}, {kNextDoneWhenMoved, kMediumCols}, {kFindingLeft, kLargeCols}, {kNextLetGo, kMediumCols}, {kNextTouchDone, kMediumCols}, {kNextTwitch, kMediumCols}, {kNextSpins, kMediumCols}, {kNextSaves, kMediumCols}, {kSettlingHandsOff, kMediumCols}, {kSpinAgain, kMediumCols}, {kSpinFirst, kMediumCols}, {kCopyThenRunAgain, kSmallCols}, {kCopyNow, kSmallCols}, {kCopyThese, kSmallCols}, {kFullReasonOnSerial, kSmallCols}, {kLabelDone, 12}, {kLabelAbort, 12}, {kLabelRunAgain, 12}, {kHeaderFlipped, kHeaderCols}, {kHeaderMechTurned, kHeaderCols}, {kHeaderSavedNew, kHeaderCols}, {kNothingSavedYet, kLargeCols}, {kFrontRule, kMediumCols}, {kFlipWhat, kMediumCols}, {kFlipAgain, kSmallCols}, {kFlipNew, kSmallCols}, {kFlipWarn, kSmallCols}, {kMechNotOnCard, kSmallCols}, {kMechSave, kSmallCols}, {kMechAgain, kSmallCols}, {kLabelNew, 12}, {kLabelSave, 12}, {kNoteFirst, kSmallCols}, {kNoteNotChecked, kSmallCols}, {kWarningCut, kSmallCols}, {kFlipWhatTyped, kMediumCols}, {kFlipNewTyped, kSmallCols}, {kFlipWarnTyped, kSmallCols}, {kNoteTypedSame, kSmallCols}, {kNoteFixTyped, kSmallCols}, {kNoteTypedPartial, kSmallCols}, }
```

EVERY literal above with its budget — the list the compile-time check walks.

*constant, declared at [`include/shulib/calibration/calibrate_panel.hpp:541`](../../include/shulib/calibration/calibrate_panel.hpp#L541).*

<a id="panelliteralsfit"></a>

## `panelLiteralsFit`

```cpp
[[nodiscard]] constexpr bool panelLiteralsFit() noexcept
```

True when every named literal and every STOPPED wording fits its budget and is ASCII with no '%' — evaluated at compile time below, so a budget change that breaks one fails the build.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:579`](../../include/shulib/calibration/calibrate_panel.hpp#L579).*

<a id="panellayoutholds"></a>

## `panelLayoutHolds`

```cpp
[[nodiscard]] constexpr bool panelLayoutHolds() noexcept
```

True when the fixed layout keeps its promises: everything on the screen above row 236; ABORT's slot inside the powered row; and no middle box (a held screen's two choices, RUN AGAIN) whose HIT box reaches the hit box of anything in the button row — so a second tap where DONE or ABORT was lands on nothing in the next layout. The two choices do not touch each other.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:602`](../../include/shulib/calibration/calibrate_panel.hpp#L602).*

<a id="struct-wrapped"></a>

## `struct Wrapped`

```cpp
template <std::size_t Cols, std::size_t Lines> struct Wrapped
```

Text broken into at most `Lines` lines of at most `Cols` characters — see wrapWords().

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:628`](../../include/shulib/calibration/calibrate_panel.hpp#L628).*

<a id="wrapped-line"></a>

### `Wrapped::line`

```cpp
char line[Lines][Cols + 1]{}
```

The lines, each NUL-terminated.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:630`](../../include/shulib/calibration/calibrate_panel.hpp#L630).*

<a id="wrapped-count"></a>

### `Wrapped::count`

```cpp
std::size_t count = 0
```

How many lines are used.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:632`](../../include/shulib/calibration/calibrate_panel.hpp#L632).*

<a id="wrapped-complete"></a>

### `Wrapped::complete`

```cpp
bool complete = true
```

False when the text did not fit: a word longer than `Cols` (it is never split), or more text than `Lines` lines hold (the lines hold what fitted, in order; the rest is NOT shown and this flag says so).

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:636`](../../include/shulib/calibration/calibrate_panel.hpp#L636).*

<a id="wrapwords"></a>

## `wrapWords`

```cpp
template <std::size_t Cols, std::size_t Lines> [[nodiscard]] constexpr Wrapped<Cols, Lines> wrapWords(std::string_view text) noexcept
```

Break `text` into lines of at most `Cols` characters, ONLY at spaces: a word — and therefore a number like 1.37500 — is never split, runs of spaces collapse to one, and nothing is dropped silently (Wrapped::complete). constexpr, so a literal's wrapping can be checked at compile time. Allocation-free.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:644`](../../include/shulib/calibration/calibrate_panel.hpp#L644).*

<a id="viewfor"></a>

## `viewFor`

```cpp
[[nodiscard]] inline PanelView viewFor(const DrivetrainCalibrator& cal, double secondsSinceLayoutChange, SaveOutcome save = SaveOutcome::NotAttempted, const FinishFacts& finish = FinishFacts{})
```

The view for this frame. `secondsSinceLayoutChange` arms the non-ABORT buttons once it reaches kArmingDelaySeconds (TouchGate::view() tracks it); `save` tells the finished screen whether the program managed to write the result. A pure function of the calibrator's public queries.  CAL1: four steps (1/4..4/4), PUSH FRONT FIRST, the front rule while the IMU warms up — the end that leads the push is the FRONT, so there is no WHICH END question (removed by CAL1 Part B). `save` Held paints the held screen `finish.hold` names — FRONT FLIPPED or MECHANISM PORT TURNED, NOTHING SAVED YET, a left box (SaveHeld, labelled USE NEW or SAVE) and RUN AGAIN (which discards the run). `finish.front` New makes a SAVED header "SAVED -- NEW FRONT" in amber; `finish.note` is one amber SMALL row on the finished screen (a calibrator warning that needs two rows is then cut to one, ending " ...").

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:800`](../../include/shulib/calibration/calibrate_panel.hpp#L800).*

<a id="hittest"></a>

## `hitTest`

```cpp
[[nodiscard]] constexpr int hitTest(const PanelView& v, int x, int y) noexcept
```

The index of the ARMED button whose hit box (hitBox()) contains (x, y), or −1. A greyed button is never hit; neither is anything outside every button.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:1090`](../../include/shulib/calibration/calibrate_panel.hpp#L1090).*

<a id="layoutkey"></a>

## `layoutKey`

```cpp
[[nodiscard]] constexpr std::uint32_t layoutKey(const PanelView& v) noexcept
```

A number that changes exactly when the LAYOUT does — the screen and every button's identity and rectangle — and NOT when a button merely arms. TouchGate restarts its arming clock on a change.

*free function, declared at [`include/shulib/calibration/calibrate_panel.hpp:1101`](../../include/shulib/calibration/calibrate_panel.hpp#L1101).*

<a id="struct-touchsample"></a>

## `struct TouchSample`

```cpp
struct TouchSample
```

One read of the touch screen, PROS-free: what screen_touch_status() returns, reduced to what a release-edge reader needs.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:1124`](../../include/shulib/calibration/calibrate_panel.hpp#L1124).*

<a id="touchsample-error"></a>

### `TouchSample::error`

```cpp
bool error = false
```

The read FAILED (PROS: E_TOUCH_ERROR, every field −1). Ignored — and it must not move the release count, or the next good read would replay the last touch.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:1127`](../../include/shulib/calibration/calibrate_panel.hpp#L1127).*

<a id="touchsample-releasecount"></a>

### `TouchSample::releaseCount`

```cpp
std::int32_t releaseCount = 0
```

PROS's release_count: it only ever increases, one per finger lifted.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:1129`](../../include/shulib/calibration/calibrate_panel.hpp#L1129).*

<a id="touchsample-x"></a>

### `TouchSample::x`

```cpp
std::int16_t x = 0
```

Where the LAST touch was — stale while idle, so it means something only on a new release.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:1131`](../../include/shulib/calibration/calibrate_panel.hpp#L1131).*

<a id="touchsample-y"></a>

### `TouchSample::y`

```cpp
std::int16_t y = 0
```

See x.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:1133`](../../include/shulib/calibration/calibrate_panel.hpp#L1133).*

<a id="enum-class-touchaction"></a>

## `enum class TouchAction`

```cpp
enum class TouchAction : std::uint8_t
```

What a touch should do.

*enum class, declared at [`include/shulib/calibration/calibrate_panel.hpp:1137`](../../include/shulib/calibration/calibrate_panel.hpp#L1137).*

<a id="touchaction-none"></a>

### `TouchAction::None`

```cpp
None
```

nothing — no new release, a miss, a greyed button, or a stale layout

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:1138`](../../include/shulib/calibration/calibrate_panel.hpp#L1138).*

<a id="touchaction-answer"></a>

### `TouchAction::Answer`

```cpp
Answer
```

send TouchResult::answer to the calibrator

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:1139`](../../include/shulib/calibration/calibrate_panel.hpp#L1139).*

<a id="touchaction-runagain"></a>

### `TouchAction::RunAgain`

```cpp
RunAgain
```

rebuild the calibrator (the program's job — drivetrain_calibrator.hpp)

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:1140`](../../include/shulib/calibration/calibrate_panel.hpp#L1140).*

<a id="touchaction-saveheld"></a>

### `TouchAction::SaveHeld`

```cpp
SaveHeld
```

(CAL1) a held screen's USE NEW / SAVE: save this run AS MEASURED

*enumerator, declared at [`include/shulib/calibration/calibrate_panel.hpp:1141`](../../include/shulib/calibration/calibrate_panel.hpp#L1141).*

<a id="struct-touchresult"></a>

## `struct TouchResult`

```cpp
struct TouchResult
```

The outcome of one touch read.

*struct, declared at [`include/shulib/calibration/calibrate_panel.hpp:1145`](../../include/shulib/calibration/calibrate_panel.hpp#L1145).*

<a id="touchresult-action"></a>

### `TouchResult::action`

```cpp
TouchAction action = TouchAction::None
```

What to do.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:1147`](../../include/shulib/calibration/calibrate_panel.hpp#L1147).*

<a id="touchresult-answer"></a>

### `TouchResult::answer`

```cpp
Answer answer = Answer::Abort
```

The answer, when action is Answer.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:1149`](../../include/shulib/calibration/calibrate_panel.hpp#L1149).*

<a id="touchresult-hit"></a>

### `TouchResult::hit`

```cpp
int hit = -1
```

The button index hit in the DRAWN view (−1 for none) — for the serial log.

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:1151`](../../include/shulib/calibration/calibrate_panel.hpp#L1151).*

<a id="touchresult-newrelease"></a>

### `TouchResult::newRelease`

```cpp
bool newRelease = false
```

True when this read carried a NEW release (logged even when it hits nothing).

*field, declared at [`include/shulib/calibration/calibrate_panel.hpp:1153`](../../include/shulib/calibration/calibrate_panel.hpp#L1153).*

<a id="class-touchgate"></a>

## `class TouchGate`

```cpp
class TouchGate
```

The touch discipline, in one place: the release EDGE only; error reads ignored and never replayed; the layout clock that arms buttons kArmingDelaySeconds after every layout change; and a re-check against the calibrator before any answer — because the view a finger landed on may be one frame older than the state machine.

*class, declared at [`include/shulib/calibration/calibrate_panel.hpp:1160`](../../include/shulib/calibration/calibrate_panel.hpp#L1160).*

<a id="touchgate-touchgate"></a>

### `TouchGate::TouchGate`

```cpp
explicit TouchGate(std::int32_t releaseCountAtStart) noexcept
```

`releaseCountAtStart`: PROS's release_count when the panel first appears, so a tap made before the program existed is never replayed as an answer.

*function, declared at [`include/shulib/calibration/calibrate_panel.hpp:1164`](../../include/shulib/calibration/calibrate_panel.hpp#L1164).*

<a id="touchgate-view"></a>

### `TouchGate::view`

```cpp
[[nodiscard]] PanelView view(const DrivetrainCalibrator& cal, double nowSeconds, SaveOutcome save = SaveOutcome::NotAttempted, const FinishFacts& finish = FinishFacts{})
```

The view to paint now. Restarts the arming clock at `nowSeconds` whenever the layout (layoutKey()) differs from the last view this returned — so the program never has to track it, and a new layout's buttons come up greyed on the very first frame they appear. `finish` is viewFor()'s (CAL1; the default is no hold, no NOTE). CAL1: on a held screen the clock ALSO restarts when the QUESTION changes — the hold's reason, or (MECHANISM PORT TURNED, one screen per port) its port: FRONT FLIPPED's USE NEW and every MECHANISM PORT TURNED's SAVE share one box and one id, so layoutKey() alone cannot see the next question arrive, and a reflexive second tap would land on the next question's live box.

*function, declared at [`include/shulib/calibration/calibrate_panel.hpp:1175`](../../include/shulib/calibration/calibrate_panel.hpp#L1175).*

<a id="touchgate-touch"></a>

### `TouchGate::touch`

```cpp
[[nodiscard]] TouchResult touch(const TouchSample& s, const PanelView& drawn, const DrivetrainCalibrator& cal)
```

One touch read against the view that is ON THE SCREEN (`drawn`). A new release that lands on an armed button becomes an answer only if the calibrator still accepts it now (DONE also needs pushReady()); RUN AGAIN only once the run has ended. CAL1: a HELD screen's left box is SaveHeld — never an answer — and only on a held screen while the run is Done.

*function, declared at [`include/shulib/calibration/calibrate_panel.hpp:1197`](../../include/shulib/calibration/calibrate_panel.hpp#L1197).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 80 lines, click to expand</summary>

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
 robot. The program (calibration/calibrate_program.hpp since CAL1; robot two's own
 src/programs/calibrate_program.cpp before CAL1 Part B) only RENDERS a PanelView.

 ── What it decides ─────────────────────────────────────────────────────────────────
  * viewFor(): per state, a header colour and word, an instruction of at most two LARGE lines,
    one big live number, a NEXT line, and the buttons that are LIVE, each with its rectangle.
  * The layout rules that make a mis-tap harmless, checked at COMPILE time on the constants and
    at test time on every reachable view:
      - ABORT sits in the SAME rectangle in every state that shows it; in the two POWERED
        phases it grows to the whole button row, which still contains that rectangle — the
        PULSES from the moment their first pulse drives a motor, not before: the PULSES are
        what a touch of DONE leads to, and DONE lies in that row (CAL1 Part B review round).
      - DONE is drawn only when DrivetrainCalibrator::pushReady() says so.
      - The held screens' two choices and RUN AGAIN are boxes in the MIDDLE of the screen, never in
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
  * (CAL1 Part A2a, 2026-10-18) the standard Calibrate's screens (calibration/calibrate_program.hpp):
    the two HELD screens, FRONT FLIPPED and MECHANISM PORT TURNED (SaveOutcome::Held,
    FinishFacts::hold; the second once per declared port that turned, FinishFacts::holdPort, its
    boxes re-greyed each time), whose left box is PanelButtonId::SaveHeld, read as
    TouchAction::SaveHeld; the SAVED header after USE NEW; and the finished screen's NOTE row.
  * (CAL1 Part B, 2026-11-05 — a BEHAVIOUR CHANGE, the team lead's ruling of 2026-10-06: the end
    that LEADS the push is the FRONT) the WHICH END question is GONE: four steps (1/4..4/4), PUSH
    FRONT FIRST, the front rule while the IMU warms up; no WhichEnd screen, no FrontRule parameter,
    no FRONT FIRST / BACK FIRST buttons. The held screen's left box is PanelButtonId::SaveHeld (the
    FRONT FIRST id until Part B), its right box RUN AGAIN; both are kChoiceLeftBox /
    kChoiceRightBox, the two middle boxes FRONT FIRST and BACK FIRST used to fill.
  * (CAL1 Part A2b, 2026-10-30 — additions, plus ONE behaviour-preserving edit) the team lead's
    D6 (a): with no usable record on the card, the program checks the front against the team's
    TYPED drive ports, and the FRONT FLIPPED screen then paints their words
    (FinishFacts::typedPorts); three NOTEs say how the front compared with them (every port
    agreed, the front agreed but some port did not, or the typed ports must be fixed after USE
    NEW). The one edit to an existing line: panelChips() copies its chips with "%.*s" of exactly
    `used` characters instead of "%s" (the same characters — GCC's -Wformat-truncation at -O1,
    the comment there; robot two's own Calibrate program, before CAL1 Part B replaced it,
    re-measured byte-identical on it in CAL1 A2b's review).

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
