<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/mechanism/tune_panel.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `tune_panel.hpp`

mechanism/tune_panel.hpp — "shulib Tune"'s brain screen as a PURE MODEL: what each page says, where each button is, which touches mean something (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's D7 (a) and D8 (a), with its re-check amendments #3, #7, #8…

This header declares **10** types (113 members), **3** free functions, and **13** constants.

Extracted from [`include/shulib/mechanism/tune_panel.hpp`](../../include/shulib/mechanism/tune_panel.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class TunePage`](#enum-class-tunepage)
  - [`Clear`](#tunepage-clear)
  - [`Tune`](#tunepage-tune)
  - [`Confirm`](#tunepage-confirm)
  - [`Report`](#tunepage-report)
  - [`StringCheck`](#tunepage-stringcheck)
  - [`Exited`](#tunepage-exited)
  - [`Stopped`](#tunepage-stopped)
- [`enum class TuneButton`](#enum-class-tunebutton)
  - [`Ready`](#tunebutton-ready)
  - [`PrevMech`](#tunebutton-prevmech)
  - [`NextMech`](#tunebutton-nextmech)
  - [`PrevParam`](#tunebutton-prevparam)
  - [`NextParam`](#tunebutton-nextparam)
  - [`Minus`](#tunebutton-minus)
  - [`Plus`](#tunebutton-plus)
  - [`Fine`](#tunebutton-fine)
  - [`TestA`](#tunebutton-testa)
  - [`TestB`](#tunebutton-testb)
  - [`Home`](#tunebutton-home)
  - [`Undo`](#tunebutton-undo)
  - [`Reset`](#tunebutton-reset)
  - [`Copy`](#tunebutton-copy)
  - [`StringCheck`](#tunebutton-stringcheck)
  - [`Stop`](#tunebutton-stop)
  - [`Exit`](#tunebutton-exit)
  - [`Go`](#tunebutton-go)
  - [`Back`](#tunebutton-back)
  - [`Again`](#tunebutton-again)
  - [`TuneAgain`](#tunebutton-tuneagain)
- [`struct TuneButtonView`](#struct-tunebuttonview)
  - [`id`](#tunebuttonview-id)
  - [`rect`](#tunebuttonview-rect)
  - [`label`](#tunebuttonview-label)
  - [`armed`](#tunebuttonview-armed)
  - [`tone`](#tunebuttonview-tone)
- [`struct TuneLine`](#struct-tuneline)
  - [`text`](#tuneline-text)
  - [`size`](#tuneline-size)
  - [`x`](#tuneline-x)
  - [`y`](#tuneline-y)
  - [`tone`](#tuneline-tone)
- [`kTuneMaxLines`](#ktunemaxlines) — *constant*
- [`kTuneMaxButtons`](#ktunemaxbuttons) — *constant*
- [`struct TuneView`](#struct-tuneview)
  - [`page`](#tuneview-page)
  - [`lines`](#tuneview-lines)
  - [`lineCount`](#tuneview-linecount)
  - [`buttons`](#tuneview-buttons)
  - [`buttonCount`](#tuneview-buttoncount)
  - [`layoutKey`](#tuneview-layoutkey)
  - [`button`](#tuneview-button)
- [`enum class TuneTest`](#enum-class-tunetest)
  - [`None`](#tunetest-none)
  - [`A`](#tunetest-a)
  - [`B`](#tunetest-b)
- [`struct TuneReport`](#struct-tunereport)
  - [`test`](#tunereport-test)
  - [`kind`](#tunereport-kind)
  - [`unit`](#tunereport-unit)
  - [`target`](#tunereport-target)
  - [`verdict`](#tunereport-verdict)
  - [`seconds`](#tunereport-seconds)
  - [`bound`](#tunereport-bound)
  - [`overshoot`](#tunereport-overshoot)
  - [`settle`](#tunereport-settle)
  - [`peakAmps`](#tunereport-peakamps)
  - [`finalError`](#tunereport-finalerror)
  - [`hasFinalError`](#tunereport-hasfinalerror)
  - [`holdAmps`](#tunereport-holdamps)
  - [`holdEnded`](#tunereport-holdended)
  - [`holdStopped`](#tunereport-holdstopped)
  - [`stallAmps`](#tunereport-stallamps)
  - [`capAmps`](#tunereport-capamps)
  - [`warnStallNotAboveHold`](#tunereport-warnstallnotabovehold)
  - [`warnCapNotAbovePeak`](#tunereport-warncapnotabovepeak)
  - [`stoppedBy`](#tunereport-stoppedby)
- [`struct TuneMechFacts`](#struct-tunemechfacts)
  - [`name`](#tunemechfacts-name)
  - [`kind`](#tunemechfacts-kind)
  - [`units`](#tunemechfacts-units)
  - [`built`](#tunemechfacts-built)
  - [`homed`](#tunemechfacts-homed)
  - [`homes`](#tunemechfacts-homes)
  - [`now`](#tunemechfacts-now)
  - [`target`](#tunemechfacts-target)
  - [`active`](#tunemechfacts-active)
  - [`lastVerdict`](#tunemechfacts-lastverdict)
  - [`changed`](#tunemechfacts-changed)
  - [`capChanged`](#tunemechfacts-capchanged)
- [`struct TuneParamFacts`](#struct-tuneparamfacts)
  - [`name`](#tuneparamfacts-name)
  - [`value`](#tuneparamfacts-value)
  - [`file`](#tuneparamfacts-file)
  - [`unit`](#tuneparamfacts-unit)
  - [`rule`](#tuneparamfacts-rule)
  - [`capNone`](#tuneparamfacts-capnone)
- [`struct TuneFacts`](#struct-tunefacts)
  - [`page`](#tunefacts-page)
  - [`armed`](#tunefacts-armed)
  - [`mechCount`](#tunefacts-mechcount)
  - [`mech`](#tunefacts-mech)
  - [`m`](#tunefacts-m)
  - [`sel`](#tunefacts-sel)
  - [`next`](#tunefacts-next)
  - [`param`](#tunefacts-param)
  - [`paramCount`](#tunefacts-paramcount)
  - [`fine`](#tunefacts-fine)
  - [`busy`](#tunefacts-busy)
  - [`undoDepth`](#tunefacts-undodepth)
  - [`controller`](#tunefacts-controller)
  - [`field`](#tunefacts-field)
  - [`buttonHeld`](#tunefacts-buttonheld)
  - [`anyCapChanged`](#tunefacts-anycapchanged)
  - [`status`](#tunefacts-status)
  - [`testA`](#tunefacts-testa)
  - [`testB`](#tunefacts-testb)
  - [`testAName`](#tunefacts-testaname)
  - [`testBName`](#tunefacts-testbname)
  - [`confirmWhat`](#tunefacts-confirmwhat)
  - [`report`](#tunefacts-report)
  - [`limitsWritten`](#tunefacts-limitswritten)
  - [`copiedAtExit`](#tunefacts-copiedatexit)
  - [`resetAtExit`](#tunefacts-resetatexit)
  - [`stoppedWhy`](#tunefacts-stoppedwhy)
- [`kPitch`](#kpitch) — *constant*
- [`kTitleY`](#ktitley) — *constant*
- [`kStateY`](#kstatey) — *constant*
- [`kSafetyY`](#ksafetyy) — *constant*
- [`kRow1`](#krow1) — *constant*
- [`kParamY`](#kparamy) — *constant*
- [`kRow2`](#krow2) — *constant*
- [`kStatusY`](#kstatusy) — *constant*
- [`kRow3`](#krow3) — *constant*
- [`kLargeTitleY`](#klargetitley) — *constant*
- [`kBodyY`](#kbodyy) — *constant*
- [`tuneLabelCols`](#tunelabelcols) — *free function*
- [`tuneView`](#tuneview) — *free function*
- [`tuneHit`](#tunehit) — *free function*

<a id="enum-class-tunepage"></a>

## `enum class TunePage`

```cpp
enum class TunePage : std::uint8_t
```

The Tune program's pages (header).

*enum class, declared at [`include/shulib/mechanism/tune_panel.hpp:67`](../../include/shulib/mechanism/tune_panel.hpp#L67).*

<a id="tunepage-clear"></a>

### `TunePage::Clear`

```cpp
Clear
```

stage 1 of the clear confirm: nothing is made yet

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:68`](../../include/shulib/mechanism/tune_panel.hpp#L68).*

<a id="tunepage-tune"></a>

### `TunePage::Tune`

```cpp
Tune
```

the tuning page

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:69`](../../include/shulib/mechanism/tune_panel.hpp#L69).*

<a id="tunepage-confirm"></a>

### `TunePage::Confirm`

```cpp
Confirm
```

stage 2: before a mechanism's first motion in the session

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:70`](../../include/shulib/mechanism/tune_panel.hpp#L70).*

<a id="tunepage-report"></a>

### `TunePage::Report`

```cpp
Report
```

a TEST move's report

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:71`](../../include/shulib/mechanism/tune_panel.hpp#L71).*

<a id="tunepage-stringcheck"></a>

### `TunePage::StringCheck`

```cpp
StringCheck
```

STRING CHECK — empty until MECH-S-B

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:72`](../../include/shulib/mechanism/tune_panel.hpp#L72).*

<a id="tunepage-exited"></a>

### `TunePage::Exited`

```cpp
Exited
```

after EXIT: limits written back, everything released

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:73`](../../include/shulib/mechanism/tune_panel.hpp#L73).*

<a id="tunepage-stopped"></a>

### `TunePage::Stopped`

```cpp
Stopped
```

a program error: everything released

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:74`](../../include/shulib/mechanism/tune_panel.hpp#L74).*

<a id="enum-class-tunebutton"></a>

## `enum class TuneButton`

```cpp
enum class TuneButton : std::uint8_t
```

Every button any page has.

*enum class, declared at [`include/shulib/mechanism/tune_panel.hpp:78`](../../include/shulib/mechanism/tune_panel.hpp#L78).*

<a id="tunebutton-ready"></a>

### `TuneButton::Ready`

```cpp
Ready
```

stage 1: make the mechanisms

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:79`](../../include/shulib/mechanism/tune_panel.hpp#L79).*

<a id="tunebutton-prevmech"></a>

### `TuneButton::PrevMech`

```cpp
PrevMech
```

the previous mechanism

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:80`](../../include/shulib/mechanism/tune_panel.hpp#L80).*

<a id="tunebutton-nextmech"></a>

### `TuneButton::NextMech`

```cpp
NextMech
```

the next mechanism

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:81`](../../include/shulib/mechanism/tune_panel.hpp#L81).*

<a id="tunebutton-prevparam"></a>

### `TuneButton::PrevParam`

```cpp
PrevParam
```

the previous number

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:82`](../../include/shulib/mechanism/tune_panel.hpp#L82).*

<a id="tunebutton-nextparam"></a>

### `TuneButton::NextParam`

```cpp
NextParam
```

the next number

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:83`](../../include/shulib/mechanism/tune_panel.hpp#L83).*

<a id="tunebutton-minus"></a>

### `TuneButton::Minus`

```cpp
Minus
```

one [−] press on the selected number

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:84`](../../include/shulib/mechanism/tune_panel.hpp#L84).*

<a id="tunebutton-plus"></a>

### `TuneButton::Plus`

```cpp
Plus
```

one [+] press on the selected number

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:85`](../../include/shulib/mechanism/tune_panel.hpp#L85).*

<a id="tunebutton-fine"></a>

### `TuneButton::Fine`

```cpp
Fine
```

FINE on/off (a gain's factor 1.05 instead of 1.25)

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:86`](../../include/shulib/mechanism/tune_panel.hpp#L86).*

<a id="tunebutton-testa"></a>

### `TuneButton::TestA`

```cpp
TestA
```

a test move to target A

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:87`](../../include/shulib/mechanism/tune_panel.hpp#L87).*

<a id="tunebutton-testb"></a>

### `TuneButton::TestB`

```cpp
TestB
```

a test move to target B

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:88`](../../include/shulib/mechanism/tune_panel.hpp#L88).*

<a id="tunebutton-home"></a>

### `TuneButton::Home`

```cpp
Home
```

the declaration's homing step

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:89`](../../include/shulib/mechanism/tune_panel.hpp#L89).*

<a id="tunebutton-undo"></a>

### `TuneButton::Undo`

```cpp
Undo
```

undo the last change

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:90`](../../include/shulib/mechanism/tune_panel.hpp#L90).*

<a id="tunebutton-reset"></a>

### `TuneButton::Reset`

```cpp
Reset
```

this mechanism back to the robot file's numbers

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:91`](../../include/shulib/mechanism/tune_panel.hpp#L91).*

<a id="tunebutton-copy"></a>

### `TuneButton::Copy`

```cpp
Copy
```

log this mechanism's COPY block

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:92`](../../include/shulib/mechanism/tune_panel.hpp#L92).*

<a id="tunebutton-stringcheck"></a>

### `TuneButton::StringCheck`

```cpp
StringCheck
```

the STRING CHECK page (a lift's)

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:93`](../../include/shulib/mechanism/tune_panel.hpp#L93).*

<a id="tunebutton-stop"></a>

### `TuneButton::Stop`

```cpp
Stop
```

stop every mechanism

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:94`](../../include/shulib/mechanism/tune_panel.hpp#L94).*

<a id="tunebutton-exit"></a>

### `TuneButton::Exit`

```cpp
Exit
```

write back every limit changed, release everything

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:95`](../../include/shulib/mechanism/tune_panel.hpp#L95).*

<a id="tunebutton-go"></a>

### `TuneButton::Go`

```cpp
Go
```

stage 2: the motion may start

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:96`](../../include/shulib/mechanism/tune_panel.hpp#L96).*

<a id="tunebutton-back"></a>

### `TuneButton::Back`

```cpp
Back
```

back to the tuning page

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:97`](../../include/shulib/mechanism/tune_panel.hpp#L97).*

<a id="tunebutton-again"></a>

### `TuneButton::Again`

```cpp
Again
```

the same test again

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:98`](../../include/shulib/mechanism/tune_panel.hpp#L98).*

<a id="tunebutton-tuneagain"></a>

### `TuneButton::TuneAgain`

```cpp
TuneAgain
```

after EXIT: back to the tuning page

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:99`](../../include/shulib/mechanism/tune_panel.hpp#L99).*

<a id="struct-tunebuttonview"></a>

## `struct TuneButtonView`

```cpp
struct TuneButtonView
```

One button as a page draws it.

*struct, declared at [`include/shulib/mechanism/tune_panel.hpp:103`](../../include/shulib/mechanism/tune_panel.hpp#L103).*

<a id="tunebuttonview-id"></a>

### `TuneButtonView::id`

```cpp
TuneButton id = TuneButton::Stop
```

Which button.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:105`](../../include/shulib/mechanism/tune_panel.hpp#L105).*

<a id="tunebuttonview-rect"></a>

### `TuneButtonView::rect`

```cpp
ui::Rect rect{}
```

Where it is drawn (its hit box is ui::hitBox() of this).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:107`](../../include/shulib/mechanism/tune_panel.hpp#L107).*

<a id="tunebuttonview-label"></a>

### `TuneButtonView::label`

```cpp
char label[24]{}
```

Its label (SMALL text, centred). Room for the longest label written in tuneView() ("READY - I AM CLEAR", 18 characters) — a 16-byte box once cut it to "READY - I AM CL"; tune_panel_test pins every label whole.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:110`](../../include/shulib/mechanism/tune_panel.hpp#L110).*

<a id="tunebuttonview-armed"></a>

### `TuneButtonView::armed`

```cpp
bool armed = false
```

It takes touches (false: greyed — not available, or the layout is still arming).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:112`](../../include/shulib/mechanism/tune_panel.hpp#L112).*

<a id="tunebuttonview-tone"></a>

### `TuneButtonView::tone`

```cpp
ui::Tone tone = ui::Tone::Blue
```

Its colour by meaning (STOP red, GO and READY green, the rest blue).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:114`](../../include/shulib/mechanism/tune_panel.hpp#L114).*

<a id="struct-tuneline"></a>

## `struct TuneLine`

```cpp
struct TuneLine
```

One text line as a page draws it.

*struct, declared at [`include/shulib/mechanism/tune_panel.hpp:118`](../../include/shulib/mechanism/tune_panel.hpp#L118).*

<a id="tuneline-text"></a>

### `TuneLine::text`

```cpp
char text[ui::kSmallCols + 1]{}
```

The text, at most kSmallCols characters (a LARGE line at most kLargeCols).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:120`](../../include/shulib/mechanism/tune_panel.hpp#L120).*

<a id="tuneline-size"></a>

### `TuneLine::size`

```cpp
ui::TextSize size = ui::TextSize::Small
```

Its size.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:122`](../../include/shulib/mechanism/tune_panel.hpp#L122).*

<a id="tuneline-x"></a>

### `TuneLine::x`

```cpp
std::int16_t x = 0
```

Its left column.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:124`](../../include/shulib/mechanism/tune_panel.hpp#L124).*

<a id="tuneline-y"></a>

### `TuneLine::y`

```cpp
std::int16_t y = 0
```

Its top row.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:126`](../../include/shulib/mechanism/tune_panel.hpp#L126).*

<a id="tuneline-tone"></a>

### `TuneLine::tone`

```cpp
ui::Tone tone = ui::Tone::White
```

Its colour by meaning.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:128`](../../include/shulib/mechanism/tune_panel.hpp#L128).*

<a id="ktunemaxlines"></a>

## `kTuneMaxLines`

```cpp
inline constexpr std::size_t kTuneMaxLines = 10
```

The most text lines a page has.

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:132`](../../include/shulib/mechanism/tune_panel.hpp#L132).*

<a id="ktunemaxbuttons"></a>

## `kTuneMaxButtons`

```cpp
inline constexpr std::size_t kTuneMaxButtons = 16
```

The most buttons a page has.

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:134`](../../include/shulib/mechanism/tune_panel.hpp#L134).*

<a id="struct-tuneview"></a>

## `struct TuneView`

```cpp
struct TuneView
```

One page as it is to be drawn: its lines and its buttons.

*struct, declared at [`include/shulib/mechanism/tune_panel.hpp:137`](../../include/shulib/mechanism/tune_panel.hpp#L137).*

<a id="tuneview-page"></a>

### `TuneView::page`

```cpp
TunePage page = TunePage::Clear
```

Which page.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:139`](../../include/shulib/mechanism/tune_panel.hpp#L139).*

<a id="tuneview-lines"></a>

### `TuneView::lines`

```cpp
std::array<TuneLine, kTuneMaxLines> lines{}
```

Its text lines.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:141`](../../include/shulib/mechanism/tune_panel.hpp#L141).*

<a id="tuneview-linecount"></a>

### `TuneView::lineCount`

```cpp
std::size_t lineCount = 0
```

How many lines are real.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:143`](../../include/shulib/mechanism/tune_panel.hpp#L143).*

<a id="tuneview-buttons"></a>

### `TuneView::buttons`

```cpp
std::array<TuneButtonView, kTuneMaxButtons> buttons{}
```

Its buttons.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:145`](../../include/shulib/mechanism/tune_panel.hpp#L145).*

<a id="tuneview-buttoncount"></a>

### `TuneView::buttonCount`

```cpp
std::size_t buttonCount = 0
```

How many buttons are real.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:147`](../../include/shulib/mechanism/tune_panel.hpp#L147).*

<a id="tuneview-layoutkey"></a>

### `TuneView::layoutKey`

```cpp
[[nodiscard]] std::uint32_t layoutKey() const noexcept
```

The LAYOUT's key: the page and every button's identity and rectangle — never whether a button is armed, never text (ui::LayoutKey). An ArmingClock restarts when it changes.

*function, declared at [`include/shulib/mechanism/tune_panel.hpp:151`](../../include/shulib/mechanism/tune_panel.hpp#L151).*

<a id="tuneview-button"></a>

### `TuneView::button`

```cpp
[[nodiscard]] const TuneButtonView* button(TuneButton id) const noexcept
```

The button with `id`, or nullptr when the page has none.

*function, declared at [`include/shulib/mechanism/tune_panel.hpp:161`](../../include/shulib/mechanism/tune_panel.hpp#L161).*

<a id="enum-class-tunetest"></a>

## `enum class TuneTest`

```cpp
enum class TuneTest : std::uint8_t
```

Which test a report is of.

*enum class, declared at [`include/shulib/mechanism/tune_panel.hpp:172`](../../include/shulib/mechanism/tune_panel.hpp#L172).*

<a id="tunetest-none"></a>

### `TuneTest::None`

```cpp
None
```

none

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:173`](../../include/shulib/mechanism/tune_panel.hpp#L173).*

<a id="tunetest-a"></a>

### `TuneTest::A`

```cpp
A
```

TEST A

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:174`](../../include/shulib/mechanism/tune_panel.hpp#L174).*

<a id="tunetest-b"></a>

### `TuneTest::B`

```cpp
B
```

TEST B

*enumerator, declared at [`include/shulib/mechanism/tune_panel.hpp:175`](../../include/shulib/mechanism/tune_panel.hpp#L175).*

<a id="struct-tunereport"></a>

## `struct TuneReport`

```cpp
struct TuneReport
```

A TEST move's report (re-check #3, #7): what the program measured from its samples (tune_program.hpp's tuneReportFrom()). A field the move cannot have is negative ("n/a").

*struct, declared at [`include/shulib/mechanism/tune_panel.hpp:180`](../../include/shulib/mechanism/tune_panel.hpp#L180).*

<a id="tunereport-test"></a>

### `TuneReport::test`

```cpp
TuneTest test = TuneTest::None
```

Which test.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:182`](../../include/shulib/mechanism/tune_panel.hpp#L182).*

<a id="tunereport-kind"></a>

### `TuneReport::kind`

```cpp
Kind kind = Kind::Position
```

The mechanism's kind.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:184`](../../include/shulib/mechanism/tune_panel.hpp#L184).*

<a id="tunereport-unit"></a>

### `TuneReport::unit`

```cpp
const char* unit = ""
```

The mechanism's unit word ("in", "deg", "rpm", "V").

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:186`](../../include/shulib/mechanism/tune_panel.hpp#L186).*

<a id="tunereport-target"></a>

### `TuneReport::target`

```cpp
double target = 0.0
```

The target (a position, an output rpm, or the volts of a power run).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:188`](../../include/shulib/mechanism/tune_panel.hpp#L188).*

<a id="tunereport-verdict"></a>

### `TuneReport::verdict`

```cpp
const char* verdict = ""
```

The verdict (manipulation::mechanismOutcomeName's word).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:190`](../../include/shulib/mechanism/tune_panel.hpp#L190).*

<a id="tunereport-seconds"></a>

### `TuneReport::seconds`

```cpp
double seconds = 0.0
```

Seconds from the command to the verdict (the service's StepReport::elapsed).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:192`](../../include/shulib/mechanism/tune_panel.hpp#L192).*

<a id="tunereport-bound"></a>

### `TuneReport::bound`

```cpp
double bound = 0.0
```

The move's time limit, seconds (re-check #7: the plan's time × 2 + the settle time + 0.5 s, at most 10 s).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:195`](../../include/shulib/mechanism/tune_panel.hpp#L195).*

<a id="tunereport-overshoot"></a>

### `TuneReport::overshoot`

```cpp
double overshoot = -1.0
```

The farthest past the target in the direction of travel, ≥ 0 (negative: n/a).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:197`](../../include/shulib/mechanism/tune_panel.hpp#L197).*

<a id="tunereport-settle"></a>

### `TuneReport::settle`

```cpp
double settle = -1.0
```

Seconds from the first sample inside "reached" (or the at-speed band) to the verdict (negative: never inside, or n/a).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:200`](../../include/shulib/mechanism/tune_panel.hpp#L200).*

<a id="tunereport-peakamps"></a>

### `TuneReport::peakAmps`

```cpp
double peakAmps = 0.0
```

The most current any of its motors drew during the move, A.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:202`](../../include/shulib/mechanism/tune_panel.hpp#L202).*

<a id="tunereport-finalerror"></a>

### `TuneReport::finalError`

```cpp
double finalError = 0.0
```

Where it ended minus the target (negative values are short of it upward; n/a for a power run — see `hasFinalError`).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:205`](../../include/shulib/mechanism/tune_panel.hpp#L205).*

<a id="tunereport-hasfinalerror"></a>

### `TuneReport::hasFinalError`

```cpp
bool hasFinalError = false
```

`finalError` means something (not a power run).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:207`](../../include/shulib/mechanism/tune_panel.hpp#L207).*

<a id="tunereport-holdamps"></a>

### `TuneReport::holdAmps`

```cpp
double holdAmps = -1.0
```

The mean current over the second after it settled, A (negative: n/a — the move did not succeed, a power run, or `holdEnded` / `holdStopped` says why).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:210`](../../include/shulib/mechanism/tune_panel.hpp#L210).*

<a id="tunereport-holdended"></a>

### `TuneReport::holdEnded`

```cpp
bool holdEnded = false
```

The hold ENDED inside its second: the stall signature tripped on the hold itself (V1) — the hold is n/a and the stall warning is set.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:213`](../../include/shulib/mechanism/tune_panel.hpp#L213).*

<a id="tunereport-holdstopped"></a>

### `TuneReport::holdStopped`

```cpp
bool holdStopped = false
```

A STOP ended the hold second (`stoppedBy` names it): the hold is n/a, with no warning of its own.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:215`](../../include/shulib/mechanism/tune_panel.hpp#L215).*

<a id="tunereport-stallamps"></a>

### `TuneReport::stallAmps`

```cpp
double stallAmps = 0.0
```

Its stall signature's amps (0: it states none).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:217`](../../include/shulib/mechanism/tune_panel.hpp#L217).*

<a id="tunereport-capamps"></a>

### `TuneReport::capAmps`

```cpp
double capAmps = 0.0
```

Its current cap, A per motor (0: none).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:219`](../../include/shulib/mechanism/tune_panel.hpp#L219).*

<a id="tunereport-warnstallnotabovehold"></a>

### `TuneReport::warnStallNotAboveHold`

```cpp
bool warnStallNotAboveHold = false
```

WARNING: the stall signature is not above the hold current — an ordinary hold can trip it.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:221`](../../include/shulib/mechanism/tune_panel.hpp#L221).*

<a id="tunereport-warncapnotabovepeak"></a>

### `TuneReport::warnCapNotAbovePeak`

```cpp
bool warnCapNotAbovePeak = false
```

WARNING: the cap is not above the peak — the cap limited the move.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:223`](../../include/shulib/mechanism/tune_panel.hpp#L223).*

<a id="tunereport-stoppedby"></a>

### `TuneReport::stoppedBy`

```cpp
const char* stoppedBy = ""
```

For a CANCELLED move: what stopped it ("a controller button"), else "".

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:225`](../../include/shulib/mechanism/tune_panel.hpp#L225).*

<a id="struct-tunemechfacts"></a>

## `struct TuneMechFacts`

```cpp
struct TuneMechFacts
```

The facts the TUNE page shows about the selected mechanism.

*struct, declared at [`include/shulib/mechanism/tune_panel.hpp:229`](../../include/shulib/mechanism/tune_panel.hpp#L229).*

<a id="tunemechfacts-name"></a>

### `TuneMechFacts::name`

```cpp
const char* name = ""
```

Its name (at most 23 characters; the program passes the spec's).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:231`](../../include/shulib/mechanism/tune_panel.hpp#L231).*

<a id="tunemechfacts-kind"></a>

### `TuneMechFacts::kind`

```cpp
Kind kind = Kind::Power
```

Its kind.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:233`](../../include/shulib/mechanism/tune_panel.hpp#L233).*

<a id="tunemechfacts-units"></a>

### `TuneMechFacts::units`

```cpp
Units units = Units::Degrees
```

A position mechanism's units.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:235`](../../include/shulib/mechanism/tune_panel.hpp#L235).*

<a id="tunemechfacts-built"></a>

### `TuneMechFacts::built`

```cpp
bool built = true
```

Built at boot (false: ABSENT).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:237`](../../include/shulib/mechanism/tune_panel.hpp#L237).*

<a id="tunemechfacts-homed"></a>

### `TuneMechFacts::homed`

```cpp
bool homed = false
```

A position mechanism has found its zero.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:239`](../../include/shulib/mechanism/tune_panel.hpp#L239).*

<a id="tunemechfacts-homes"></a>

### `TuneMechFacts::homes`

```cpp
bool homes = false
```

A position mechanism has a homing step (`.home` other than neverHomed).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:241`](../../include/shulib/mechanism/tune_panel.hpp#L241).*

<a id="tunemechfacts-now"></a>

### `TuneMechFacts::now`

```cpp
double now = 0.0
```

Where it is (a position mechanism, its units) or how fast (a flywheel, output rpm), or the volts commanded (a power mechanism).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:244`](../../include/shulib/mechanism/tune_panel.hpp#L244).*

<a id="tunemechfacts-target"></a>

### `TuneMechFacts::target`

```cpp
double target = 0.0
```

What it aims at (a position's hold or move target, a flywheel's target rpm; 0 when none).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:246`](../../include/shulib/mechanism/tune_panel.hpp#L246).*

<a id="tunemechfacts-active"></a>

### `TuneMechFacts::active`

```cpp
bool active = false
```

A flywheel is spinning / a position mechanism holds or moves.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:248`](../../include/shulib/mechanism/tune_panel.hpp#L248).*

<a id="tunemechfacts-lastverdict"></a>

### `TuneMechFacts::lastVerdict`

```cpp
const char* lastVerdict = ""
```

Its last verdict's word ("" before any).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:250`](../../include/shulib/mechanism/tune_panel.hpp#L250).*

<a id="tunemechfacts-changed"></a>

### `TuneMechFacts::changed`

```cpp
bool changed = false
```

Any of its numbers differ from the robot file's.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:252`](../../include/shulib/mechanism/tune_panel.hpp#L252).*

<a id="tunemechfacts-capchanged"></a>

### `TuneMechFacts::capChanged`

```cpp
bool capChanged = false
```

Its current cap differs from the robot file's (re-check #8: a cap set here stays in the motor).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:254`](../../include/shulib/mechanism/tune_panel.hpp#L254).*

<a id="struct-tuneparamfacts"></a>

## `struct TuneParamFacts`

```cpp
struct TuneParamFacts
```

One live number as the TUNE page shows it.

*struct, declared at [`include/shulib/mechanism/tune_panel.hpp:258`](../../include/shulib/mechanism/tune_panel.hpp#L258).*

<a id="tuneparamfacts-name"></a>

### `TuneParamFacts::name`

```cpp
const char* name = nullptr
```

Its page name ("kP"; live_tuning.hpp's Parameter::name). Null: no number (a pneumatic).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:260`](../../include/shulib/mechanism/tune_panel.hpp#L260).*

<a id="tuneparamfacts-value"></a>

### `TuneParamFacts::value`

```cpp
double value = 0.0
```

Its live value.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:262`](../../include/shulib/mechanism/tune_panel.hpp#L262).*

<a id="tuneparamfacts-file"></a>

### `TuneParamFacts::file`

```cpp
double file = 0.0
```

The robot file's value.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:264`](../../include/shulib/mechanism/tune_panel.hpp#L264).*

<a id="tuneparamfacts-unit"></a>

### `TuneParamFacts::unit`

```cpp
const char* unit = ""
```

Its unit word (tunableUnit()).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:266`](../../include/shulib/mechanism/tune_panel.hpp#L266).*

<a id="tuneparamfacts-rule"></a>

### `TuneParamFacts::rule`

```cpp
StepRule rule = StepRule::Gain
```

How a press moves it.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:268`](../../include/shulib/mechanism/tune_panel.hpp#L268).*

<a id="tuneparamfacts-capnone"></a>

### `TuneParamFacts::capNone`

```cpp
bool capNone = false
```

It is a current cap the mechanism does not state (live and in the file): the page says how one starts ([−] from none sets the motor's 2.5 A less one step; [+] from none is above the motor's most and refused — live_tuning.hpp's step rule).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:272`](../../include/shulib/mechanism/tune_panel.hpp#L272).*

<a id="struct-tunefacts"></a>

## `struct TuneFacts`

```cpp
struct TuneFacts
```

Everything a page shows, as plain facts (the program fills it every frame).

*struct, declared at [`include/shulib/mechanism/tune_panel.hpp:276`](../../include/shulib/mechanism/tune_panel.hpp#L276).*

<a id="tunefacts-page"></a>

### `TuneFacts::page`

```cpp
TunePage page = TunePage::Clear
```

Which page.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:278`](../../include/shulib/mechanism/tune_panel.hpp#L278).*

<a id="tunefacts-armed"></a>

### `TuneFacts::armed`

```cpp
bool armed = false
```

The layout has been up long enough to take touches (ui::ArmingClock::armedAfter).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:280`](../../include/shulib/mechanism/tune_panel.hpp#L280).*

<a id="tunefacts-mechcount"></a>

### `TuneFacts::mechCount`

```cpp
std::size_t mechCount = 0
```

How many mechanisms the robot declares.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:282`](../../include/shulib/mechanism/tune_panel.hpp#L282).*

<a id="tunefacts-mech"></a>

### `TuneFacts::mech`

```cpp
std::size_t mech = 0
```

Which one is shown (0-based).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:284`](../../include/shulib/mechanism/tune_panel.hpp#L284).*

<a id="tunefacts-m"></a>

### `TuneFacts::m`

```cpp
TuneMechFacts m{}
```

The shown mechanism.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:286`](../../include/shulib/mechanism/tune_panel.hpp#L286).*

<a id="tunefacts-sel"></a>

### `TuneFacts::sel`

```cpp
TuneParamFacts sel{}
```

The selected number.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:288`](../../include/shulib/mechanism/tune_panel.hpp#L288).*

<a id="tunefacts-next"></a>

### `TuneFacts::next`

```cpp
TuneParamFacts next{}
```

The number after it (for the "next:" line), name null at the end of the list.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:290`](../../include/shulib/mechanism/tune_panel.hpp#L290).*

<a id="tunefacts-param"></a>

### `TuneFacts::param`

```cpp
std::size_t param = 0
```

Which number is selected (0-based) and how many the mechanism has.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:292`](../../include/shulib/mechanism/tune_panel.hpp#L292).*

<a id="tunefacts-paramcount"></a>

### `TuneFacts::paramCount`

```cpp
std::size_t paramCount = 0
```

How many numbers the mechanism has live.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:294`](../../include/shulib/mechanism/tune_panel.hpp#L294).*

<a id="tunefacts-fine"></a>

### `TuneFacts::fine`

```cpp
bool fine = false
```

FINE is on.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:296`](../../include/shulib/mechanism/tune_panel.hpp#L296).*

<a id="tunefacts-busy"></a>

### `TuneFacts::busy`

```cpp
bool busy = false
```

A test move, a homing or a test's hold second is running: every button but STOP is greyed.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:298`](../../include/shulib/mechanism/tune_panel.hpp#L298).*

<a id="tunefacts-undodepth"></a>

### `TuneFacts::undoDepth`

```cpp
std::size_t undoDepth = 0
```

How many changes UNDO can reach.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:300`](../../include/shulib/mechanism/tune_panel.hpp#L300).*

<a id="tunefacts-controller"></a>

### `TuneFacts::controller`

```cpp
bool controller = false
```

The controller is connected.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:302`](../../include/shulib/mechanism/tune_panel.hpp#L302).*

<a id="tunefacts-field"></a>

### `TuneFacts::field`

```cpp
bool field = false
```

VEXos reports a field or a competition switch plugged in.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:304`](../../include/shulib/mechanism/tune_panel.hpp#L304).*

<a id="tunefacts-buttonheld"></a>

### `TuneFacts::buttonHeld`

```cpp
bool buttonHeld = false
```

A controller button is held (every button is a STOP): READY waits.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:306`](../../include/shulib/mechanism/tune_panel.hpp#L306).*

<a id="tunefacts-anycapchanged"></a>

### `TuneFacts::anyCapChanged`

```cpp
bool anyCapChanged = false
```

ANY mechanism's live current cap differs from the robot file's (the TUNE page's safety line: a cap set here stays in the motor — EXIT before you stop).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:309`](../../include/shulib/mechanism/tune_panel.hpp#L309).*

<a id="tunefacts-status"></a>

### `TuneFacts::status`

```cpp
const char* status = ""
```

The last sentence (an applied change, a refusal, a verdict): wrapped onto two lines, cut there.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:311`](../../include/shulib/mechanism/tune_panel.hpp#L311).*

<a id="tunefacts-testa"></a>

### `TuneFacts::testA`

```cpp
double testA = 0.0
```

TEST A's target in the mechanism's unit, and its name ("floor", or "1/4" of the range).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:313`](../../include/shulib/mechanism/tune_panel.hpp#L313).*

<a id="tunefacts-testb"></a>

### `TuneFacts::testB`

```cpp
double testB = 0.0
```

TEST B's target.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:315`](../../include/shulib/mechanism/tune_panel.hpp#L315).*

<a id="tunefacts-testaname"></a>

### `TuneFacts::testAName`

```cpp
const char* testAName = ""
```

TEST A's name.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:317`](../../include/shulib/mechanism/tune_panel.hpp#L317).*

<a id="tunefacts-testbname"></a>

### `TuneFacts::testBName`

```cpp
const char* testBName = ""
```

TEST B's name.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:319`](../../include/shulib/mechanism/tune_panel.hpp#L319).*

<a id="tunefacts-confirmwhat"></a>

### `TuneFacts::confirmWhat`

```cpp
const char* confirmWhat = ""
```

CONFIRM: what will move it ("TEST A", "HOME", "the stick").

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:321`](../../include/shulib/mechanism/tune_panel.hpp#L321).*

<a id="tunefacts-report"></a>

### `TuneFacts::report`

```cpp
TuneReport report{}
```

REPORT: the report.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:323`](../../include/shulib/mechanism/tune_panel.hpp#L323).*

<a id="tunefacts-limitswritten"></a>

### `TuneFacts::limitsWritten`

```cpp
int limitsWritten = 0
```

EXITED: how many limits were written back.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:325`](../../include/shulib/mechanism/tune_panel.hpp#L325).*

<a id="tunefacts-copiedatexit"></a>

### `TuneFacts::copiedAtExit`

```cpp
int copiedAtExit = 0
```

EXITED: how many mechanisms' changed numbers EXIT COPIED to the log first (never copied since their last change).

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:328`](../../include/shulib/mechanism/tune_panel.hpp#L328).*

<a id="tunefacts-resetatexit"></a>

### `TuneFacts::resetAtExit`

```cpp
int resetAtExit = 0
```

EXITED: how many mechanisms EXIT had to RESET (a live signature at or above the file's cap, so the cap alone could not go back) — each COPIED first.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:331`](../../include/shulib/mechanism/tune_panel.hpp#L331).*

<a id="tunefacts-stoppedwhy"></a>

### `TuneFacts::stoppedWhy`

```cpp
const char* stoppedWhy = ""
```

STOPPED: why.

*field, declared at [`include/shulib/mechanism/tune_panel.hpp:333`](../../include/shulib/mechanism/tune_panel.hpp#L333).*

<a id="kpitch"></a>

## `kPitch`

```cpp
inline constexpr std::int16_t kPitch = ui::kSmallHeight
```

The SMALL line pitch on every page, pixels (ui::kSmallHeight).

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:338`](../../include/shulib/mechanism/tune_panel.hpp#L338).*

<a id="ktitley"></a>

## `kTitleY`

```cpp
inline constexpr std::int16_t kTitleY = 0
```

The tuning page's three top lines.

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:340`](../../include/shulib/mechanism/tune_panel.hpp#L340).*

<a id="kstatey"></a>

## `kStateY`

```cpp
inline constexpr std::int16_t kStateY = 16
```

The state line.

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:342`](../../include/shulib/mechanism/tune_panel.hpp#L342).*

<a id="ksafetyy"></a>

## `kSafetyY`

```cpp
inline constexpr std::int16_t kSafetyY = 32
```

The safety line (the controller, the field, a cap left changed).

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:344`](../../include/shulib/mechanism/tune_panel.hpp#L344).*

<a id="krow1"></a>

## `kRow1`

```cpp
inline constexpr ui::Rect kRow1{0, 52, 479, 81}
```

Row 1 (‹ › ^ v − + FINE).

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:346`](../../include/shulib/mechanism/tune_panel.hpp#L346).*

<a id="kparamy"></a>

## `kParamY`

```cpp
inline constexpr std::int16_t kParamY = 86
```

The selected number's three lines.

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:348`](../../include/shulib/mechanism/tune_panel.hpp#L348).*

<a id="krow2"></a>

## `kRow2`

```cpp
inline constexpr ui::Rect kRow2{0, 136, 479, 165}
```

Row 2 (TEST A, TEST B, HOME, UNDO, RESET, COPY).

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:350`](../../include/shulib/mechanism/tune_panel.hpp#L350).*

<a id="kstatusy"></a>

## `kStatusY`

```cpp
inline constexpr std::int16_t kStatusY = 169
```

The two status lines.

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:352`](../../include/shulib/mechanism/tune_panel.hpp#L352).*

<a id="krow3"></a>

## `kRow3`

```cpp
inline constexpr ui::Rect kRow3{0, 204, 479, 235}
```

Row 3 (STRING, STOP, EXIT) — and the other pages' one row of buttons.

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:354`](../../include/shulib/mechanism/tune_panel.hpp#L354).*

<a id="klargetitley"></a>

## `kLargeTitleY`

```cpp
inline constexpr std::int16_t kLargeTitleY = 2
```

A LARGE title's top row on the other pages.

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:356`](../../include/shulib/mechanism/tune_panel.hpp#L356).*

<a id="kbodyy"></a>

## `kBodyY`

```cpp
inline constexpr std::int16_t kBodyY = 42
```

The other pages' first SMALL line.

*constant, declared at [`include/shulib/mechanism/tune_panel.hpp:358`](../../include/shulib/mechanism/tune_panel.hpp#L358).*

<a id="tunelabelcols"></a>

## `tuneLabelCols`

```cpp
[[nodiscard]] constexpr std::size_t tuneLabelCols(ui::Rect r) noexcept
```

The SMALL characters a button `r` fits (at the width the SMALL budget implies over 472 px).

*free function, declared at [`include/shulib/mechanism/tune_panel.hpp:472`](../../include/shulib/mechanism/tune_panel.hpp#L472).*

<a id="tuneview"></a>

## `tuneView`

```cpp
[[nodiscard]] inline TuneView tuneView(const TuneFacts& f) noexcept
```

THE PAGE for `f` (header): every word, every button and whether it is armed. Pure.

*free function, declared at [`include/shulib/mechanism/tune_panel.hpp:478`](../../include/shulib/mechanism/tune_panel.hpp#L478).*

<a id="tunehit"></a>

## `tuneHit`

```cpp
[[nodiscard]] inline bool tuneHit(const TuneView& v, int x, int y, TuneButton& out) noexcept
```

The button an accepted touch at (x, y) hits on `v` — only an ARMED one (ui::hitTest) — or false.

*free function, declared at [`include/shulib/mechanism/tune_panel.hpp:760`](../../include/shulib/mechanism/tune_panel.hpp#L760).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 49 lines, click to expand</summary>

```text

 mechanism/tune_panel.hpp — "shulib Tune"'s brain screen as a PURE MODEL: what each page says, where
 each button is, which touches mean something (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's
 D7 (a) and D8 (a), with its re-check amendments #3, #7, #8 and #14).

 ── Why a model ─────────────────────────────────────────────────────────────────────
 The Tune program (mechanism/tune_program.hpp) is how a team tunes a mechanism ON THE ROBOT: a
 cascade lift's gains, its top speed, its homing push and its current cap, found at the bench by a
 student who is not a programmer. What the screen says and where its buttons are is what decides
 whether that works, so it is host-tested here, page by page, at every mechanism kind and the
 longest name a mechanism can have (23 characters) — the house's pattern since R3d Part 2F-1
 (calibration/calibrate_panel.hpp) and COMP-a (ui/touch_panel.hpp, whose budgets this uses: one set
 of budgets for every screen, all ESTIMATES until the SCREEN RULER is read — A4 register HA-136).

 ── The pages ───────────────────────────────────────────────────────────────────────
   CLEAR        stage 1 of the clear confirm (re-check #7): NOTHING is made before READY — making a
                3-wire line drives it to its start level (HA-119: robot two's claw moves at boot).
                READY is greyed, with why in red, while no controller is connected, a field or switch
                is plugged in or a controller button is held.
   TUNE         the mechanism (‹ › picks one), its state, the selected number with [−] [+] and
                FINE, TEST A / TEST B, HOME, UNDO, RESET, COPY, STRING, STOP and EXIT.
   CONFIRM      stage 2 of the clear confirm: before a mechanism's FIRST motion in the session
                (a TEST, a HOME or the stick), GO or BACK.
   REPORT       a TEST move's report: the verdict, the time, the farthest past the target, the
                settle, the most current, the final error, the hold current over the second after
                it settled (or WHY there is none: the hold ENDED, it was stopped), and a WARNING when
                the stall signature is not above that hold or the cap not above the peak (re-check #3:
                the four currents' order). A power run shows no settle and no hold (n/a). Its STOP is
                where the TUNE page's is, never greyed (the page flips here as a test ends).
   STRING CHECK EMPTY in this run: MECH-S-B (after Nov 21) fills it.
   EXITED       after EXIT: every limit this program changed written back, every mechanism released,
                and what EXIT COPIED (changed numbers never copied) or RESET (a cap that could not go
                back alone) on the way.
   STOPPED      a program error: everything released, the error's words, nothing more.

 ── The rules every page keeps (tested in test/tune_panel_test.cpp) ─────────────────
 Every rectangle inside 480 × 236; every pair of buttons further apart than twice the hit pads (so
 no padded hit box overlaps another); no text line under a button; every text within its size's
 budget; printable ASCII with no '%' (ui::screenText — PROS hands text to VEXos as a FORMAT string).
 Every word a page shows is built HERE from plain facts the program hands in (TuneFacts), so the
 budgets hold for every value a mechanism can have: numbers are printed %.5g (at most 11
 characters), a status sentence is wrapped onto two lines and cut there (the program logs it whole).
 A new layout's buttons are dead for ui::kArmingDelaySeconds (a reflexive second tap lands on
 nothing) — the program says so in TuneFacts::armed — EXCEPT the STOP of the tuning and REPORT pages
 (the same cell on both), which is always armed: a reflexive tap on it only stops everything, and the
 page a test starts on (after GO) must be stoppable from its first tick.

 PROS-free, allocation-free, header-only; nothing here draws, reads a touch or knows a clock.
 NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
