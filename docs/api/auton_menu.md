<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/competition/auton_menu.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `auton_menu.hpp`

competition::AutonMenu — the brain-screen auton selector as a PURE MODEL.

This header declares **7** types (46 members) and **18** constants.

Extracted from [`include/shulib/competition/auton_menu.hpp`](../../include/shulib/competition/auton_menu.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMenuEntriesPerPage`](#kmenuentriesperpage) — *constant*
- [`kMenuMaxButtons`](#kmenumaxbuttons) — *constant*
- [`kMenuHeaderBar`](#kmenuheaderbar) — *constant*
- [`kMenuEntryRow0`](#kmenuentryrow0) — *constant*
- [`kMenuEntryRow1`](#kmenuentryrow1) — *constant*
- [`kMenuPrevSlot`](#kmenuprevslot) — *constant*
- [`kMenuNoneSlot`](#kmenunoneslot) — *constant*
- [`kMenuNextSlot`](#kmenunextslot) — *constant*
- [`kMenuEntryRows`](#kmenuentryrows) — *constant*
- [`kMenuHeaderPrefix`](#kmenuheaderprefix) — *constant*
- [`kMenuNoneLabel`](#kmenunonelabel) — *constant*
- [`kMenuPrevLabel`](#kmenuprevlabel) — *constant*
- [`kMenuNextLabel`](#kmenunextlabel) — *constant*
- [`kMenuHintText`](#kmenuhinttext) — *constant*
- [`kMenuLockedText`](#kmenulockedtext) — *constant*
- [`kMenuNoneInfo`](#kmenunoneinfo) — *constant*
- [`kMenuRefusedPrefix`](#kmenurefusedprefix) — *constant*
- [`kMenuRefusalTurnSeconds`](#kmenurefusalturnseconds) — *constant*
- [`enum class MenuButtonId`](#enum-class-menubuttonid)
  - [`Entry`](#menubuttonid-entry)
  - [`None`](#menubuttonid-none)
  - [`Prev`](#menubuttonid-prev)
  - [`Next`](#menubuttonid-next)
- [`struct MenuButton`](#struct-menubutton)
  - [`id`](#menubutton-id)
  - [`entry`](#menubutton-entry)
  - [`rect`](#menubutton-rect)
  - [`label`](#menubutton-label)
  - [`armed`](#menubutton-armed)
  - [`chosen`](#menubutton-chosen)
- [`struct MenuLine`](#struct-menuline)
  - [`x`](#menuline-x)
  - [`y`](#menuline-y)
  - [`size`](#menuline-size)
  - [`tone`](#menuline-tone)
  - [`text`](#menuline-text)
- [`struct MenuView`](#struct-menuview)
  - [`header`](#menuview-header)
  - [`info`](#menuview-info)
  - [`status`](#menuview-status)
  - [`buttons`](#menuview-buttons)
  - [`buttonCount`](#menuview-buttoncount)
  - [`page`](#menuview-page)
  - [`pageCount`](#menuview-pagecount)
  - [`locked`](#menuview-locked)
- [`enum class MenuTouchAction`](#enum-class-menutouchaction)
  - [`None`](#menutouchaction-none)
  - [`Chose`](#menutouchaction-chose)
  - [`Paged`](#menutouchaction-paged)
  - [`RefusedLocked`](#menutouchaction-refusedlocked)
- [`struct MenuTouch`](#struct-menutouch)
  - [`action`](#menutouch-action)
  - [`hit`](#menutouch-hit)
  - [`newRelease`](#menutouch-newrelease)
  - [`choice`](#menutouch-choice)
- [`class AutonMenu`](#class-autonmenu)
  - [`AutonMenu`](#autonmenu-autonmenu)
  - [`AutonMenu (overload 2)`](#autonmenu-autonmenu-2)
  - [`operator=`](#autonmenu-operator-eq)
  - [`AutonMenu (overload 3)`](#autonmenu-autonmenu-3)
  - [`operator= (overload 2)`](#autonmenu-operator-eq-2)
  - [`~AutonMenu`](#autonmenu-destructor-autonmenu)
  - [`view`](#autonmenu-view)
  - [`touch`](#autonmenu-touch)
  - [`choice`](#autonmenu-choice)
  - [`chosenEntry`](#autonmenu-chosenentry)
  - [`lockChoice`](#autonmenu-lockchoice)
  - [`unlock`](#autonmenu-unlock)
  - [`locked`](#autonmenu-locked)
  - [`page`](#autonmenu-page)
  - [`pageCount`](#autonmenu-pagecount)

<a id="kmenuentriesperpage"></a>

## `kMenuEntriesPerPage`

```cpp
inline constexpr std::size_t kMenuEntriesPerPage = 2
```

How many auton buttons one page shows. Two full-width rows fit between the two status lines and the bottom row with every vertical gap above 2 * ui::kHitPadY (the layout rule that keeps two padded hit boxes from overlapping). The layout is an ESTIMATE like every screen number (HA-136).

*constant, declared at [`include/shulib/competition/auton_menu.hpp:59`](../../include/shulib/competition/auton_menu.hpp#L59).*

<a id="kmenumaxbuttons"></a>

## `kMenuMaxButtons`

```cpp
inline constexpr std::size_t kMenuMaxButtons = kMenuEntriesPerPage + 3
```

The most buttons one view carries: the page's autons plus PREV, NONE and NEXT.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:61`](../../include/shulib/competition/auton_menu.hpp#L61).*

<a id="kmenuheaderbar"></a>

## `kMenuHeaderBar`

```cpp
inline constexpr ui::Rect kMenuHeaderBar{0, 0, 479, 26}
```

The header bar (the current choice).

*constant, declared at [`include/shulib/competition/auton_menu.hpp:64`](../../include/shulib/competition/auton_menu.hpp#L64).*

<a id="kmenuentryrow0"></a>

## `kMenuEntryRow0`

```cpp
inline constexpr ui::Rect kMenuEntryRow0{6, 66, 474, 100}
```

The first auton row: full width, so a 29-character name and its kind fit one medium line.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:66`](../../include/shulib/competition/auton_menu.hpp#L66).*

<a id="kmenuentryrow1"></a>

## `kMenuEntryRow1`

```cpp
inline constexpr ui::Rect kMenuEntryRow1{6, 122, 474, 156}
```

The second auton row.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:68`](../../include/shulib/competition/auton_menu.hpp#L68).*

<a id="kmenuprevslot"></a>

## `kMenuPrevSlot`

```cpp
inline constexpr ui::Rect kMenuPrevSlot{6, 180, 154, 234}
```

PREV, bottom left — drawn only when there is a previous page.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:70`](../../include/shulib/competition/auton_menu.hpp#L70).*

<a id="kmenunoneslot"></a>

## `kMenuNoneSlot`

```cpp
inline constexpr ui::Rect kMenuNoneSlot{166, 180, 314, 234}
```

NONE, bottom centre — drawn on EVERY view, always in this place.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:72`](../../include/shulib/competition/auton_menu.hpp#L72).*

<a id="kmenunextslot"></a>

## `kMenuNextSlot`

```cpp
inline constexpr ui::Rect kMenuNextSlot{326, 180, 474, 234}
```

NEXT, bottom right — drawn only when there is a next page.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:74`](../../include/shulib/competition/auton_menu.hpp#L74).*

<a id="kmenuentryrows"></a>

## `kMenuEntryRows`

```cpp
inline constexpr ui::Rect kMenuEntryRows[kMenuEntriesPerPage] = {kMenuEntryRow0, kMenuEntryRow1}
```

The auton rows, top to bottom.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:77`](../../include/shulib/competition/auton_menu.hpp#L77).*

<a id="kmenuheaderprefix"></a>

## `kMenuHeaderPrefix`

```cpp
inline constexpr std::string_view kMenuHeaderPrefix = "AUTON: "
```

The header's prefix; "AUTON: " + a kMaxAutonNameChars name is one medium line (B25).

*constant, declared at [`include/shulib/competition/auton_menu.hpp:80`](../../include/shulib/competition/auton_menu.hpp#L80).*

<a id="kmenunonelabel"></a>

## `kMenuNoneLabel`

```cpp
inline constexpr std::string_view kMenuNoneLabel = "NONE"
```

NONE's label.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:82`](../../include/shulib/competition/auton_menu.hpp#L82).*

<a id="kmenuprevlabel"></a>

## `kMenuPrevLabel`

```cpp
inline constexpr std::string_view kMenuPrevLabel = "< PREV"
```

PREV's label.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:84`](../../include/shulib/competition/auton_menu.hpp#L84).*

<a id="kmenunextlabel"></a>

## `kMenuNextLabel`

```cpp
inline constexpr std::string_view kMenuNextLabel = "NEXT >"
```

NEXT's label.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:86`](../../include/shulib/competition/auton_menu.hpp#L86).*

<a id="kmenuhinttext"></a>

## `kMenuHintText`

```cpp
inline constexpr std::string_view kMenuHintText = "touch a name to choose it; NONE runs nothing"
```

The status line while nothing is locked.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:88`](../../include/shulib/competition/auton_menu.hpp#L88).*

<a id="kmenulockedtext"></a>

## `kMenuLockedText`

```cpp
inline constexpr std::string_view kMenuLockedText = "LOCKED -- the auton has started"
```

The status line once the auton has started.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:90`](../../include/shulib/competition/auton_menu.hpp#L90).*

<a id="kmenunoneinfo"></a>

## `kMenuNoneInfo`

```cpp
inline constexpr std::string_view kMenuNoneInfo = "NONE: no auton runs (the default)"
```

The info line for NONE.

*constant, declared at [`include/shulib/competition/auton_menu.hpp:92`](../../include/shulib/competition/auton_menu.hpp#L92).*

<a id="kmenurefusedprefix"></a>

## `kMenuRefusedPrefix`

```cpp
inline constexpr std::string_view kMenuRefusedPrefix = "REFUSED "
```

The status line's prefix before a dropped entry's sentence (file banner).

*constant, declared at [`include/shulib/competition/auton_menu.hpp:94`](../../include/shulib/competition/auton_menu.hpp#L94).*

<a id="kmenurefusalturnseconds"></a>

## `kMenuRefusalTurnSeconds`

```cpp
inline constexpr double kMenuRefusalTurnSeconds = 2.0
```

How long the status line names each dropped entry before the next (file banner).

*constant, declared at [`include/shulib/competition/auton_menu.hpp:96`](../../include/shulib/competition/auton_menu.hpp#L96).*

<a id="enum-class-menubuttonid"></a>

## `enum class MenuButtonId`

```cpp
enum class MenuButtonId : std::uint8_t
```

Which button.

*enum class, declared at [`include/shulib/competition/auton_menu.hpp:123`](../../include/shulib/competition/auton_menu.hpp#L123).*

<a id="menubuttonid-entry"></a>

### `MenuButtonId::Entry`

```cpp
Entry
```

a registered auton (MenuButton::entry says which)

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:124`](../../include/shulib/competition/auton_menu.hpp#L124).*

<a id="menubuttonid-none"></a>

### `MenuButtonId::None`

```cpp
None
```

NONE — always present, never delayed

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:125`](../../include/shulib/competition/auton_menu.hpp#L125).*

<a id="menubuttonid-prev"></a>

### `MenuButtonId::Prev`

```cpp
Prev
```

the previous page

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:126`](../../include/shulib/competition/auton_menu.hpp#L126).*

<a id="menubuttonid-next"></a>

### `MenuButtonId::Next`

```cpp
Next
```

the next page

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:127`](../../include/shulib/competition/auton_menu.hpp#L127).*

<a id="struct-menubutton"></a>

## `struct MenuButton`

```cpp
struct MenuButton
```

One button of a view.

*struct, declared at [`include/shulib/competition/auton_menu.hpp:131`](../../include/shulib/competition/auton_menu.hpp#L131).*

<a id="menubutton-id"></a>

### `MenuButton::id`

```cpp
MenuButtonId id = MenuButtonId::None
```

Which button.

*field, declared at [`include/shulib/competition/auton_menu.hpp:133`](../../include/shulib/competition/auton_menu.hpp#L133).*

<a id="menubutton-entry"></a>

### `MenuButton::entry`

```cpp
int entry = -1
```

For an Entry: the registry index it chooses. -1 otherwise.

*field, declared at [`include/shulib/competition/auton_menu.hpp:135`](../../include/shulib/competition/auton_menu.hpp#L135).*

<a id="menubutton-rect"></a>

### `MenuButton::rect`

```cpp
ui::Rect rect{}
```

Where it is drawn (its hit box is ui::hitBox(rect)).

*field, declared at [`include/shulib/competition/auton_menu.hpp:137`](../../include/shulib/competition/auton_menu.hpp#L137).*

<a id="menubutton-label"></a>

### `MenuButton::label`

```cpp
char label[ui::kMediumCols + 1]{}
```

Its label, MEDIUM, screen-safe.

*field, declared at [`include/shulib/competition/auton_menu.hpp:139`](../../include/shulib/competition/auton_menu.hpp#L139).*

<a id="menubutton-armed"></a>

### `MenuButton::armed`

```cpp
bool armed = true
```

False while the arming delay runs or once the choice is locked: drawn GREYED, never hit.

*field, declared at [`include/shulib/competition/auton_menu.hpp:141`](../../include/shulib/competition/auton_menu.hpp#L141).*

<a id="menubutton-chosen"></a>

### `MenuButton::chosen`

```cpp
bool chosen = false
```

True for the button whose choice is the current one (drawn highlighted).

*field, declared at [`include/shulib/competition/auton_menu.hpp:143`](../../include/shulib/competition/auton_menu.hpp#L143).*

<a id="struct-menuline"></a>

## `struct MenuLine`

```cpp
struct MenuLine
```

One line of text at a fixed place and size.

*struct, declared at [`include/shulib/competition/auton_menu.hpp:147`](../../include/shulib/competition/auton_menu.hpp#L147).*

<a id="menuline-x"></a>

### `MenuLine::x`

```cpp
std::int16_t x = 0
```

Left edge, pixels.

*field, declared at [`include/shulib/competition/auton_menu.hpp:149`](../../include/shulib/competition/auton_menu.hpp#L149).*

<a id="menuline-y"></a>

### `MenuLine::y`

```cpp
std::int16_t y = 0
```

Top edge, pixels.

*field, declared at [`include/shulib/competition/auton_menu.hpp:151`](../../include/shulib/competition/auton_menu.hpp#L151).*

<a id="menuline-size"></a>

### `MenuLine::size`

```cpp
ui::TextSize size = ui::TextSize::Medium
```

Its size; the text fits ui::budgetFor(size).

*field, declared at [`include/shulib/competition/auton_menu.hpp:153`](../../include/shulib/competition/auton_menu.hpp#L153).*

<a id="menuline-tone"></a>

### `MenuLine::tone`

```cpp
ui::Tone tone = ui::Tone::White
```

Its colour, by meaning.

*field, declared at [`include/shulib/competition/auton_menu.hpp:155`](../../include/shulib/competition/auton_menu.hpp#L155).*

<a id="menuline-text"></a>

### `MenuLine::text`

```cpp
char text[ui::kSmallCols + 1]{}
```

The text, NUL-terminated, screen-safe.

*field, declared at [`include/shulib/competition/auton_menu.hpp:157`](../../include/shulib/competition/auton_menu.hpp#L157).*

<a id="struct-menuview"></a>

## `struct MenuView`

```cpp
struct MenuView
```

Everything one frame of the selector says, and which touches mean something.

*struct, declared at [`include/shulib/competition/auton_menu.hpp:161`](../../include/shulib/competition/auton_menu.hpp#L161).*

<a id="menuview-header"></a>

### `MenuView::header`

```cpp
MenuLine header{}
```

"AUTON: <name>" — the current choice — MEDIUM, in the header bar.

*field, declared at [`include/shulib/competition/auton_menu.hpp:163`](../../include/shulib/competition/auton_menu.hpp#L163).*

<a id="menuview-info"></a>

### `MenuView::info`

```cpp
MenuLine info{}
```

The choice's kind, schedule and start mode (or what NONE means), SMALL.

*field, declared at [`include/shulib/competition/auton_menu.hpp:165`](../../include/shulib/competition/auton_menu.hpp#L165).*

<a id="menuview-status"></a>

### `MenuView::status`

```cpp
MenuLine status{}
```

The hint, or LOCKED, and the page — SMALL.

*field, declared at [`include/shulib/competition/auton_menu.hpp:167`](../../include/shulib/competition/auton_menu.hpp#L167).*

<a id="menuview-buttons"></a>

### `MenuView::buttons`

```cpp
MenuButton buttons[kMenuMaxButtons]{}
```

The buttons, in drawing order: the page's autons, then PREV (if any), NONE, NEXT (if any).

*field, declared at [`include/shulib/competition/auton_menu.hpp:169`](../../include/shulib/competition/auton_menu.hpp#L169).*

<a id="menuview-buttoncount"></a>

### `MenuView::buttonCount`

```cpp
std::uint8_t buttonCount = 0
```

How many entries of `buttons` are real.

*field, declared at [`include/shulib/competition/auton_menu.hpp:171`](../../include/shulib/competition/auton_menu.hpp#L171).*

<a id="menuview-page"></a>

### `MenuView::page`

```cpp
int page = 0
```

The page shown, 0-based.

*field, declared at [`include/shulib/competition/auton_menu.hpp:173`](../../include/shulib/competition/auton_menu.hpp#L173).*

<a id="menuview-pagecount"></a>

### `MenuView::pageCount`

```cpp
int pageCount = 1
```

How many pages there are (at least 1 — the page with only NONE when nothing is registered).

*field, declared at [`include/shulib/competition/auton_menu.hpp:175`](../../include/shulib/competition/auton_menu.hpp#L175).*

<a id="menuview-locked"></a>

### `MenuView::locked`

```cpp
bool locked = false
```

The choice is locked: every button is dead.

*field, declared at [`include/shulib/competition/auton_menu.hpp:177`](../../include/shulib/competition/auton_menu.hpp#L177).*

<a id="enum-class-menutouchaction"></a>

## `enum class MenuTouchAction`

```cpp
enum class MenuTouchAction : std::uint8_t
```

What a touch did.

*enum class, declared at [`include/shulib/competition/auton_menu.hpp:181`](../../include/shulib/competition/auton_menu.hpp#L181).*

<a id="menutouchaction-none"></a>

### `MenuTouchAction::None`

```cpp
None
```

nothing — no new release, a miss, or a greyed button

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:182`](../../include/shulib/competition/auton_menu.hpp#L182).*

<a id="menutouchaction-chose"></a>

### `MenuTouchAction::Chose`

```cpp
Chose
```

the choice changed (or was re-chosen) — MenuTouch::choice holds it

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:183`](../../include/shulib/competition/auton_menu.hpp#L183).*

<a id="menutouchaction-paged"></a>

### `MenuTouchAction::Paged`

```cpp
Paged
```

the page changed

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:184`](../../include/shulib/competition/auton_menu.hpp#L184).*

<a id="menutouchaction-refusedlocked"></a>

### `MenuTouchAction::RefusedLocked`

```cpp
RefusedLocked
```

a live-looking button was hit after the choice locked — nothing changed

*enumerator, declared at [`include/shulib/competition/auton_menu.hpp:185`](../../include/shulib/competition/auton_menu.hpp#L185).*

<a id="struct-menutouch"></a>

## `struct MenuTouch`

```cpp
struct MenuTouch
```

The outcome of one touch read.

*struct, declared at [`include/shulib/competition/auton_menu.hpp:189`](../../include/shulib/competition/auton_menu.hpp#L189).*

<a id="menutouch-action"></a>

### `MenuTouch::action`

```cpp
MenuTouchAction action = MenuTouchAction::None
```

What happened.

*field, declared at [`include/shulib/competition/auton_menu.hpp:191`](../../include/shulib/competition/auton_menu.hpp#L191).*

<a id="menutouch-hit"></a>

### `MenuTouch::hit`

```cpp
int hit = -1
```

The button index hit in the DRAWN view (-1 for none) — for the log.

*field, declared at [`include/shulib/competition/auton_menu.hpp:193`](../../include/shulib/competition/auton_menu.hpp#L193).*

<a id="menutouch-newrelease"></a>

### `MenuTouch::newRelease`

```cpp
bool newRelease = false
```

True when this read carried a NEW release (logged even when it hits nothing).

*field, declared at [`include/shulib/competition/auton_menu.hpp:195`](../../include/shulib/competition/auton_menu.hpp#L195).*

<a id="menutouch-choice"></a>

### `MenuTouch::choice`

```cpp
int choice = -1
```

The choice after this touch: a registry index, or -1 for NONE.

*field, declared at [`include/shulib/competition/auton_menu.hpp:197`](../../include/shulib/competition/auton_menu.hpp#L197).*

<a id="class-autonmenu"></a>

## `class AutonMenu`

```cpp
class AutonMenu
```

The selector (file banner). Borrows the registry, which must be COMPLETE before the first view() and outlive the menu. view() and touch() belong to ONE task (the one that paints); choice(), chosenEntry(), lockChoice(), unlock() and locked() may be called from any task.

*class, declared at [`include/shulib/competition/auton_menu.hpp:203`](../../include/shulib/competition/auton_menu.hpp#L203).*

<a id="autonmenu-autonmenu"></a>

### `AutonMenu::AutonMenu`

```cpp
AutonMenu(const AutonRegistry& registry, std::int32_t releaseCountAtStart) noexcept
```

`releaseCountAtStart`: PROS's release_count when the menu first appears, so a tap made before it existed is never replayed as a choice. The choice starts at NONE.

*function, declared at [`include/shulib/competition/auton_menu.hpp:207`](../../include/shulib/competition/auton_menu.hpp#L207).*

<a id="autonmenu-autonmenu-2"></a>

### `AutonMenu::AutonMenu (overload 2)`

```cpp
AutonMenu(const AutonMenu&) = delete
```

Not copyable or movable: the choice is an atomic other tasks read by address.

*function, declared at [`include/shulib/competition/auton_menu.hpp:211`](../../include/shulib/competition/auton_menu.hpp#L211).*

<a id="autonmenu-operator-eq"></a>

### `AutonMenu::operator=`

```cpp
AutonMenu& operator=(const AutonMenu&) = delete
```

Not copyable or movable: the choice is an atomic other tasks read by address. See the copy constructor.

*function, declared at [`include/shulib/competition/auton_menu.hpp:213`](../../include/shulib/competition/auton_menu.hpp#L213).*

<a id="autonmenu-autonmenu-3"></a>

### `AutonMenu::AutonMenu (overload 3)`

```cpp
AutonMenu(AutonMenu&&) = delete
```

Not copyable or movable: the choice is an atomic other tasks read by address. See the copy constructor. See the copy constructor.

*function, declared at [`include/shulib/competition/auton_menu.hpp:215`](../../include/shulib/competition/auton_menu.hpp#L215).*

<a id="autonmenu-operator-eq-2"></a>

### `AutonMenu::operator= (overload 2)`

```cpp
AutonMenu& operator=(AutonMenu&&) = delete
```

Not copyable or movable: the choice is an atomic other tasks read by address. See the copy constructor. See the copy constructor. See the copy constructor.

*function, declared at [`include/shulib/competition/auton_menu.hpp:217`](../../include/shulib/competition/auton_menu.hpp#L217).*

<a id="autonmenu-destructor-autonmenu"></a>

### `AutonMenu::~AutonMenu`

```cpp
~AutonMenu() = default
```

Not copyable or movable: the choice is an atomic other tasks read by address. See the copy constructor. See the copy constructor. See the copy constructor. Owns nothing that needs releasing.

*function, declared at [`include/shulib/competition/auton_menu.hpp:219`](../../include/shulib/competition/auton_menu.hpp#L219).*

<a id="autonmenu-view"></a>

### `AutonMenu::view`

```cpp
[[nodiscard]] MenuView view(double nowSeconds)
```

The view to paint at `nowSeconds`. Restarts the arming clock whenever the layout (page and buttons) differs from the last view this returned, so a new layout's buttons come up greyed on the very first frame they appear.

*function, declared at [`include/shulib/competition/auton_menu.hpp:224`](../../include/shulib/competition/auton_menu.hpp#L224).*

<a id="autonmenu-touch"></a>

### `AutonMenu::touch`

```cpp
[[nodiscard]] MenuTouch touch(const ui::TouchSample& s, const MenuView& drawn)
```

One touch read against the view that is ON THE SCREEN (`drawn`). A new release on an armed button chooses its auton (or NONE) or flips the page; once locked, nothing changes.

*function, declared at [`include/shulib/competition/auton_menu.hpp:232`](../../include/shulib/competition/auton_menu.hpp#L232).*

<a id="autonmenu-choice"></a>

### `AutonMenu::choice`

```cpp
[[nodiscard]] int choice() const noexcept
```

The current choice: a registry index, or -1 for NONE (the default). Any task.

*function, declared at [`include/shulib/competition/auton_menu.hpp:273`](../../include/shulib/competition/auton_menu.hpp#L273).*

<a id="autonmenu-chosenentry"></a>

### `AutonMenu::chosenEntry`

```cpp
[[nodiscard]] const AutonEntry* chosenEntry() const noexcept
```

The chosen entry, or nullptr for NONE. Any task.

*function, declared at [`include/shulib/competition/auton_menu.hpp:276`](../../include/shulib/competition/auton_menu.hpp#L276).*

<a id="autonmenu-lockchoice"></a>

### `AutonMenu::lockChoice`

```cpp
int lockChoice() noexcept
```

LOCK the choice and return it, in ONE atomic operation — the auton that runs is the choice at this instant, and no touch after it changes the choice (file banner). Any task.

*function, declared at [`include/shulib/competition/auton_menu.hpp:283`](../../include/shulib/competition/auton_menu.hpp#L283).*

<a id="autonmenu-unlock"></a>

### `AutonMenu::unlock`

```cpp
void unlock() noexcept
```

Unlock: touches may change the choice again. For the program to call only after the auton it locked for has FINISHED (a second autonomous() on the same boot then takes the choice current at ITS start). Any task.

*function, declared at [`include/shulib/competition/auton_menu.hpp:288`](../../include/shulib/competition/auton_menu.hpp#L288).*

<a id="autonmenu-locked"></a>

### `AutonMenu::locked`

```cpp
[[nodiscard]] bool locked() const noexcept
```

True while locked. Any task.

*function, declared at [`include/shulib/competition/auton_menu.hpp:291`](../../include/shulib/competition/auton_menu.hpp#L291).*

<a id="autonmenu-page"></a>

### `AutonMenu::page`

```cpp
[[nodiscard]] int page() const noexcept
```

The page shown, 0-based.

*function, declared at [`include/shulib/competition/auton_menu.hpp:294`](../../include/shulib/competition/auton_menu.hpp#L294).*

<a id="autonmenu-pagecount"></a>

### `AutonMenu::pageCount`

```cpp
[[nodiscard]] int pageCount() const noexcept
```

How many pages the registry needs (at least 1).

*function, declared at [`include/shulib/competition/auton_menu.hpp:297`](../../include/shulib/competition/auton_menu.hpp#L297).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 41 lines</summary>

```text

 competition::AutonMenu — the brain-screen auton selector as a PURE MODEL (chunk COMP-a,
 2026-09-26; the COMP brief's ruling C4).

 ── What it decides, and why it is a model ──────────────────────────────────────────
 The selector is the last thing a person touches before a match, usually in a hurry, often with
 gloves, on a resistive screen whose release point wanders. Everything that decides WHAT it says
 and WHICH touches mean something lives here, where the host suite can walk every reachable view
 — the calibrate panel's lesson (calibration/calibrate_panel.hpp), built on the same generic
 pieces (ui/touch_panel.hpp). The program only RENDERS a MenuView and hands in touch samples.

 The rules, each pinned by a test:
   * NONE is ALWAYS on the screen, in the SAME place, and it is the DEFAULT: a program that boots
     and is never touched runs no auton. (A robot that drives off the line because nobody chose
     is the failure; a robot that sits still is the safe one.)
   * One button per registered auton, labelled "<name> <KIND>", PAGED when more than
     kMenuEntriesPerPage are registered; paging hides none (every entry is on exactly one page).
   * No new button appears under the one just pressed: entries live in the middle of the screen,
     NONE / PREV / NEXT in the bottom row, and a page flip either keeps a button where it was (the
     same id) or leaves that place empty — so a reflexive double tap cannot answer a question the
     first tap asked.
   * After every layout change the new buttons are greyed and dead for ui::kArmingDelaySeconds,
     except NONE, which is never delayed (choosing no auton is always safe).
   * The CHOICE is one atomic value, readable from any task; lockChoice() freezes it and returns
     it in the same atomic operation, so a touch racing the auton's start can never change the
     auton that runs — and, once locked, no touch changes it at all (every button is dead).
   * Every string it draws passes ui::screenText(): printable ASCII, no '%', within its budget —
     literals by static_assert, names because the registry refused anything else.
   * An entry the registry DROPPED (OnRefusal::DropEntry, the team lead's ruling of 2026-10-02)
     is never a button — it was never registered — and the status line NAMES it, in red, while
     the choice is not locked: `REFUSED auton '<name>': <why>`, cut at the line's budget (the
     program's log has the whole sentence), each dropped entry in turn, kMenuRefusalTurnSeconds
     apiece. A person choosing an auton must not be left to wonder why theirs is missing.

 ── What it does NOT decide ─────────────────────────────────────────────────────────
 Which task polls the screen, when to lock and unlock (the program: lock when the auton task
 starts the auton, unlock — if it wants a second autonomous() to take a new choice — only after
 that task has finished), pixels, fonts, RGB, and repainting only what changed. NOTHING HERE HAS
 RUN ON A ROBOT.

 PROS-free, allocation-free, header-only.
```

</details>
