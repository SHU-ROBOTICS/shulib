<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/record_store.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `record_store.hpp`

The calibration record's SAFE SAVE and its LOAD (chunk SAFE-SAVE Part A, 2026-09-29): a save designed so a failure part-way does not leave the card without a good record — given HA-166 (4) and (5), short of a stated two-fault limit — a load that falls back…

This header declares **12** types (67 members), **7** free functions, and **4** constants.

Extracted from [`include/shulib/calibration/record_store.hpp`](../../include/shulib/calibration/record_store.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class Which`](#enum-class-which)
  - [`Primary`](#which-primary)
  - [`Backup`](#which-backup)
  - [`None`](#which-none)
- [`chooseEffective`](#chooseeffective) — *free function*
- [`kStoreReadSlackBytes`](#kstorereadslackbytes) — *constant*
- [`kMaxStoredFileBytes`](#kmaxstoredfilebytes) — *constant*
- [`kFileReasonBytes`](#kfilereasonbytes) — *constant*
- [`struct FileRead`](#struct-fileread)
  - [`tried`](#fileread-tried)
  - [`opened`](#fileread-opened)
  - [`count`](#fileread-count)
  - [`accepted`](#fileread-accepted)
  - [`why`](#fileread-why)
  - [`path`](#fileread-path)
- [`struct BackupLoad`](#struct-backupload)
  - [`from`](#backupload-from)
  - [`primary`](#backupload-primary)
  - [`backup`](#backupload-backup)
  - [`count`](#backupload-count)
  - [`bytes`](#backupload-bytes)
- [`loadWithBackup`](#loadwithbackup) — *free function*
- [`enum class SaveResult`](#enum-class-saveresult)
  - [`Saved`](#saveresult-saved)
  - [`NotSaved`](#saveresult-notsaved)
  - [`Unconfirmed`](#saveresult-unconfirmed)
- [`enum class SaveRefusal`](#enum-class-saverefusal)
  - [`None`](#saverefusal-none)
  - [`NoCard`](#saverefusal-nocard)
  - [`BytesRefused`](#saverefusal-bytesrefused)
  - [`DescriptionInvalid`](#saverefusal-descriptioninvalid)
  - [`EncodeFailed`](#saverefusal-encodefailed)
- [`enum class UnconfirmedWhy`](#enum-class-unconfirmedwhy)
  - [`None`](#unconfirmedwhy-none)
  - [`CardGone`](#unconfirmedwhy-cardgone)
  - [`NothingOpened`](#unconfirmedwhy-nothingopened)
  - [`ReadBackDiffers`](#unconfirmedwhy-readbackdiffers)
- [`enum class FileHolds`](#enum-class-fileholds)
  - [`NotMeasured`](#fileholds-notmeasured)
  - [`Absent`](#fileholds-absent)
  - [`NotAccepted`](#fileholds-notaccepted)
  - [`New`](#fileholds-new)
  - [`Earlier`](#fileholds-earlier)
  - [`Other`](#fileholds-other)
- [`struct PutRecord`](#struct-putrecord)
  - [`attempted`](#putrecord-attempted)
  - [`reported`](#putrecord-reported)
  - [`readOpened`](#putrecord-readopened)
  - [`readCount`](#putrecord-readcount)
  - [`verified`](#putrecord-verified)
- [`struct SaveReport`](#struct-savereport)
  - [`outcome`](#savereport-outcome)
  - [`refused`](#savereport-refused)
  - [`why`](#savereport-why)
  - [`length`](#savereport-length)
  - [`before`](#savereport-before)
  - [`backupFirst`](#savereport-backupfirst)
  - [`primaryWrite`](#savereport-primarywrite)
  - [`restore`](#savereport-restore)
  - [`backupRefresh`](#savereport-backuprefresh)
  - [`unconfirmed`](#savereport-unconfirmed)
  - [`primaryDisagreed`](#savereport-primarydisagreed)
  - [`backupDisagreed`](#savereport-backupdisagreed)
  - [`effective`](#savereport-effective)
  - [`primaryAfter`](#savereport-primaryafter)
  - [`backupAfter`](#savereport-backupafter)
  - [`primaryPath`](#savereport-primarypath)
  - [`backupPath`](#savereport-backuppath)
  - [`primaryOpenedForWriting`](#savereport-primaryopenedforwriting)
- [`safeReplace`](#safereplace) — *free function*
- [`acceptRecord`](#acceptrecord) — *free function*
- [`saveRecordSafely`](#saverecordsafely) — *free function*
- [`struct RecordFileRead`](#struct-recordfileread)
  - [`tried`](#recordfileread-tried)
  - [`opened`](#recordfileread-opened)
  - [`count`](#recordfileread-count)
  - [`decoded`](#recordfileread-decoded)
  - [`why`](#recordfileread-why)
  - [`path`](#recordfileread-path)
- [`struct LoadedRecord`](#struct-loadedrecord)
  - [`from`](#loadedrecord-from)
  - [`description`](#loadedrecord-description)
  - [`primary`](#loadedrecord-primary)
  - [`backup`](#loadedrecord-backup)
- [`loadRecord`](#loadrecord) — *free function*
- [`enum class BootReadsBackup`](#enum-class-bootreadsbackup)
  - [`No`](#bootreadsbackup-no)
  - [`Yes`](#bootreadsbackup-yes)
- [`kSaveReportLineBytes`](#ksavereportlinebytes) — *constant*
- [`formatSaveReport`](#formatsavereport) — *free function*

<a id="enum-class-which"></a>

## `enum class Which`

```cpp
enum class Which : std::uint8_t
```

Which of the two files a load uses, or a save measured as the one the next boot will use.

*enum class, declared at [`include/shulib/calibration/record_store.hpp:119`](../../include/shulib/calibration/record_store.hpp#L119).*

<a id="which-primary"></a>

### `Which::Primary`

```cpp
Primary
```

the primary file (for the record: shulib_drivetrain.bin)

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:120`](../../include/shulib/calibration/record_store.hpp#L120).*

<a id="which-backup"></a>

### `Which::Backup`

```cpp
Backup
```

the backup copy (for the record: shulib_drivetrain.bak)

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:121`](../../include/shulib/calibration/record_store.hpp#L121).*

<a id="which-none"></a>

### `Which::None`

```cpp
None
```

neither is usable

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:122`](../../include/shulib/calibration/record_store.hpp#L122).*

<a id="chooseeffective"></a>

## `chooseEffective`

```cpp
[[nodiscard]] constexpr Which chooseEffective(bool primaryAccepted, bool backupAccepted) noexcept
```

THE load rule, in one place: the primary file if it is accepted, else the backup if it is accepted, else neither. The save's first and last reads and loadWithBackup() all use it.

*free function, declared at [`include/shulib/calibration/record_store.hpp:127`](../../include/shulib/calibration/record_store.hpp#L127).*

<a id="kstorereadslackbytes"></a>

## `kStoreReadSlackBytes`

```cpp
inline constexpr std::size_t kStoreReadSlackBytes = 32
```

Bytes a store's read buffer holds beyond the largest file of its kind, so a LONGER file shows up as a length refusal instead of "the first MaxBytes bytes happened to fit".

*constant, declared at [`include/shulib/calibration/record_store.hpp:133`](../../include/shulib/calibration/record_store.hpp#L133).*

<a id="kmaxstoredfilebytes"></a>

## `kMaxStoredFileBytes`

```cpp
inline constexpr std::size_t kMaxStoredFileBytes = 1024
```

The largest file safeReplace() and loadWithBackup() will put on a stack (header: STACK).

*constant, declared at [`include/shulib/calibration/record_store.hpp:136`](../../include/shulib/calibration/record_store.hpp#L136).*

<a id="kfilereasonbytes"></a>

## `kFileReasonBytes`

```cpp
inline constexpr std::size_t kFileReasonBytes = 192
```

Capacity of a refusal reason, terminator included — the size the boot's messages already use.

*constant, declared at [`include/shulib/calibration/record_store.hpp:139`](../../include/shulib/calibration/record_store.hpp#L139).*

<a id="struct-fileread"></a>

## `struct FileRead`

```cpp
struct FileRead
```

One file, as a load saw it — generic over the file's kind.

*struct, declared at [`include/shulib/calibration/record_store.hpp:142`](../../include/shulib/calibration/record_store.hpp#L142).*

<a id="fileread-tried"></a>

### `FileRead::tried`

```cpp
bool tried = false
```

The load read this file (the backup only when the primary was not accepted).

*field, declared at [`include/shulib/calibration/record_store.hpp:144`](../../include/shulib/calibration/record_store.hpp#L144).*

<a id="fileread-opened"></a>

### `FileRead::opened`

```cpp
bool opened = false
```

False: no card, or no such file (or the open was refused).

*field, declared at [`include/shulib/calibration/record_store.hpp:146`](../../include/shulib/calibration/record_store.hpp#L146).*

<a id="fileread-count"></a>

### `FileRead::count`

```cpp
std::size_t count = 0
```

Bytes read; 0 on an open but EMPTY file, and 0 when the read did not open.

*field, declared at [`include/shulib/calibration/record_store.hpp:148`](../../include/shulib/calibration/record_store.hpp#L148).*

<a id="fileread-accepted"></a>

### `FileRead::accepted`

```cpp
bool accepted = false
```

accept() took it (never for an empty file, never for a read that did not open).

*field, declared at [`include/shulib/calibration/record_store.hpp:150`](../../include/shulib/calibration/record_store.hpp#L150).*

<a id="fileread-why"></a>

### `FileRead::why`

```cpp
char why[kFileReasonBytes] = ""
```

accept()'s refusal, when the file opened, held bytes and was not accepted; "" otherwise.

*field, declared at [`include/shulib/calibration/record_store.hpp:152`](../../include/shulib/calibration/record_store.hpp#L152).*

<a id="fileread-path"></a>

### `FileRead::path`

```cpp
char path[hal::kStorePathBytes] = ""
```

The path the store's read used (the adapter's path() on the brain), for messages only.

*field, declared at [`include/shulib/calibration/record_store.hpp:154`](../../include/shulib/calibration/record_store.hpp#L154).*

<a id="struct-backupload"></a>

## `struct BackupLoad`

```cpp
template <std::size_t MaxBytes> struct BackupLoad
```

What loadWithBackup() found: which file it chose, what it saw of each, and the chosen bytes. A return value, so it lives in the CALLER's frame — MaxBytes + 640 bytes on a V5 (header: STACK).

*struct, declared at [`include/shulib/calibration/record_store.hpp:161`](../../include/shulib/calibration/record_store.hpp#L161).*

<a id="backupload-from"></a>

### `BackupLoad::from`

```cpp
Which from = Which::None
```

Primary, Backup, or None — decided by chooseEffective().

*field, declared at [`include/shulib/calibration/record_store.hpp:163`](../../include/shulib/calibration/record_store.hpp#L163).*

<a id="backupload-primary"></a>

### `BackupLoad::primary`

```cpp
FileRead primary{}
```

The primary file as the load saw it (always tried).

*field, declared at [`include/shulib/calibration/record_store.hpp:165`](../../include/shulib/calibration/record_store.hpp#L165).*

<a id="backupload-backup"></a>

### `BackupLoad::backup`

```cpp
FileRead backup{}
```

The backup as the load saw it (tried only when the primary was not accepted).

*field, declared at [`include/shulib/calibration/record_store.hpp:167`](../../include/shulib/calibration/record_store.hpp#L167).*

<a id="backupload-count"></a>

### `BackupLoad::count`

```cpp
std::size_t count = 0
```

The chosen file's length; 0 when `from` is None.

*field, declared at [`include/shulib/calibration/record_store.hpp:169`](../../include/shulib/calibration/record_store.hpp#L169).*

<a id="backupload-bytes"></a>

### `BackupLoad::bytes`

```cpp
std::array<std::byte, MaxBytes + kStoreReadSlackBytes> bytes{}
```

The chosen file's bytes, first `count` of them; meaningful unless `from` is None.

*field, declared at [`include/shulib/calibration/record_store.hpp:171`](../../include/shulib/calibration/record_store.hpp#L171).*

<a id="loadwithbackup"></a>

## `loadWithBackup`

```cpp
template <std::size_t MaxBytes, typename Accept> [[nodiscard]] BackupLoad<MaxBytes> loadWithBackup(hal::IFileStore& store, const char* primary, const char* backup, Accept&& accept)
```

THE BYTE-LEVEL LOAD. Reads `primary`; reads `backup` ONLY when the primary is not accepted; chooses by chooseEffective(). Generic over the file's kind (the record's is loadRecord(); a sibling file of up to kMaxStoredFileBytes reuses this). Not noexcept: preconditions (a null name, or two names FAT's rule makes one file — hal::sameFileOnCard()) and the store's own throws propagate; every refusal by the card is in the result.

*free function, declared at [`include/shulib/calibration/record_store.hpp:209`](../../include/shulib/calibration/record_store.hpp#L209).*

<a id="enum-class-saveresult"></a>

## `enum class SaveResult`

```cpp
enum class SaveResult : std::uint8_t
```

A save's outcome word. (Not calibrate_panel.hpp's SaveOutcome, which is the PANEL's three states; a caller maps Saved to that one's Saved, and both others to its NotSaved.)

*enum class, declared at [`include/shulib/calibration/record_store.hpp:238`](../../include/shulib/calibration/record_store.hpp#L238).*

<a id="saveresult-saved"></a>

### `SaveResult::Saved`

```cpp
Saved
```

measured: the next boot uses the primary file, and it holds the new bytes

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:239`](../../include/shulib/calibration/record_store.hpp#L239).*

<a id="saveresult-notsaved"></a>

### `SaveResult::NotSaved`

```cpp
NotSaved
```

measured: the next boot does NOT get the new bytes (the report says what)

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:240`](../../include/shulib/calibration/record_store.hpp#L240).*

<a id="saveresult-unconfirmed"></a>

### `SaveResult::Unconfirmed`

```cpp
Unconfirmed
```

the card could not be measured (u1-u3): nothing is claimed

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:241`](../../include/shulib/calibration/record_store.hpp#L241).*

<a id="enum-class-saverefusal"></a>

## `enum class SaveRefusal`

```cpp
enum class SaveRefusal : std::uint8_t
```

Why a save stopped before touching the card (nothing was opened), or None.

*enum class, declared at [`include/shulib/calibration/record_store.hpp:245`](../../include/shulib/calibration/record_store.hpp#L245).*

<a id="saverefusal-none"></a>

### `SaveRefusal::None`

```cpp
None
```

the save touched the card

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:246`](../../include/shulib/calibration/record_store.hpp#L246).*

<a id="saverefusal-nocard"></a>

### `SaveRefusal::NoCard`

```cpp
NoCard
```

cardPresent() was false at the start

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:247`](../../include/shulib/calibration/record_store.hpp#L247).*

<a id="saverefusal-bytesrefused"></a>

### `SaveRefusal::BytesRefused`

```cpp
BytesRefused
```

accept() refused the bytes to save (the reason is in `why`)

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:248`](../../include/shulib/calibration/record_store.hpp#L248).*

<a id="saverefusal-descriptioninvalid"></a>

### `SaveRefusal::DescriptionInvalid`

```cpp
DescriptionInvalid
```

saveRecordSafely(): the description's valid() refused it (`why`)

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:249`](../../include/shulib/calibration/record_store.hpp#L249).*

<a id="saverefusal-encodefailed"></a>

### `SaveRefusal::EncodeFailed`

```cpp
EncodeFailed
```

saveRecordSafely(): the description would not encode (`why`)

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:250`](../../include/shulib/calibration/record_store.hpp#L250).*

<a id="enum-class-unconfirmedwhy"></a>

## `enum class UnconfirmedWhy`

```cpp
enum class UnconfirmedWhy : std::uint8_t
```

Why a save is Unconfirmed (header: step 4), or None.

*enum class, declared at [`include/shulib/calibration/record_store.hpp:254`](../../include/shulib/calibration/record_store.hpp#L254).*

<a id="unconfirmedwhy-none"></a>

### `UnconfirmedWhy::None`

```cpp
None
```

the save was measured

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:255`](../../include/shulib/calibration/record_store.hpp#L255).*

<a id="unconfirmedwhy-cardgone"></a>

### `UnconfirmedWhy::CardGone`

```cpp
CardGone
```

(u1) cardPresent() is false at the end

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:256`](../../include/shulib/calibration/record_store.hpp#L256).*

<a id="unconfirmedwhy-nothingopened"></a>

### `UnconfirmedWhy::NothingOpened`

```cpp
NothingOpened
```

(u2) no operation in the whole save got a file open

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:257`](../../include/shulib/calibration/record_store.hpp#L257).*

<a id="unconfirmedwhy-readbackdiffers"></a>

### `UnconfirmedWhy::ReadBackDiffers`

```cpp
ReadBackDiffers
```

(u3) a file did not read back as the save last saw it

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:258`](../../include/shulib/calibration/record_store.hpp#L258).*

<a id="enum-class-fileholds"></a>

## `enum class FileHolds`

```cpp
enum class FileHolds : std::uint8_t
```

What a file held at the save's final read (step 4), relative to the save's two records.

*enum class, declared at [`include/shulib/calibration/record_store.hpp:262`](../../include/shulib/calibration/record_store.hpp#L262).*

<a id="fileholds-notmeasured"></a>

### `FileHolds::NotMeasured`

```cpp
NotMeasured
```

step 4 did not run (the save stopped before touching the card)

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:263`](../../include/shulib/calibration/record_store.hpp#L263).*

<a id="fileholds-absent"></a>

### `FileHolds::Absent`

```cpp
Absent
```

the read did not open

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:264`](../../include/shulib/calibration/record_store.hpp#L264).*

<a id="fileholds-notaccepted"></a>

### `FileHolds::NotAccepted`

```cpp
NotAccepted
```

it opened and was empty, short, long or refused by accept()

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:265`](../../include/shulib/calibration/record_store.hpp#L265).*

<a id="fileholds-new"></a>

### `FileHolds::New`

```cpp
New
```

accepted, byte-identical to this save's bytes

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:266`](../../include/shulib/calibration/record_store.hpp#L266).*

<a id="fileholds-earlier"></a>

### `FileHolds::Earlier`

```cpp
Earlier
```

accepted, byte-identical to the record in effect before the save

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:267`](../../include/shulib/calibration/record_store.hpp#L267).*

<a id="fileholds-other"></a>

### `FileHolds::Other`

```cpp
Other
```

accepted, and neither

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:268`](../../include/shulib/calibration/record_store.hpp#L268).*

<a id="struct-putrecord"></a>

## `struct PutRecord`

```cpp
struct PutRecord
```

One put(): a replace, then ALWAYS a read-back. `reported` is for the words only — `verified` alone says whether the file holds the bytes (header).

*struct, declared at [`include/shulib/calibration/record_store.hpp:273`](../../include/shulib/calibration/record_store.hpp#L273).*

<a id="putrecord-attempted"></a>

### `PutRecord::attempted`

```cpp
bool attempted = false
```

The save issued this put.

*field, declared at [`include/shulib/calibration/record_store.hpp:275`](../../include/shulib/calibration/record_store.hpp#L275).*

<a id="putrecord-reported"></a>

### `PutRecord::reported`

```cpp
hal::ReplaceStep reported = hal::ReplaceStep::None
```

What the replace reported (hal::ReplaceStep). Never used to decide anything.

*field, declared at [`include/shulib/calibration/record_store.hpp:277`](../../include/shulib/calibration/record_store.hpp#L277).*

<a id="putrecord-readopened"></a>

### `PutRecord::readOpened`

```cpp
bool readOpened = false
```

The read-back opened.

*field, declared at [`include/shulib/calibration/record_store.hpp:279`](../../include/shulib/calibration/record_store.hpp#L279).*

<a id="putrecord-readcount"></a>

### `PutRecord::readCount`

```cpp
std::size_t readCount = 0
```

Bytes the read-back returned.

*field, declared at [`include/shulib/calibration/record_store.hpp:281`](../../include/shulib/calibration/record_store.hpp#L281).*

<a id="putrecord-verified"></a>

### `PutRecord::verified`

```cpp
bool verified = false
```

The read-back returned exactly the bytes written, and accept() took them.

*field, declared at [`include/shulib/calibration/record_store.hpp:283`](../../include/shulib/calibration/record_store.hpp#L283).*

<a id="struct-savereport"></a>

## `struct SaveReport`

```cpp
struct SaveReport
```

What a safe save did and what it MEASURED. Every field is filled from what happened and from the final read of both files — none from what a step meant to do.

*struct, declared at [`include/shulib/calibration/record_store.hpp:288`](../../include/shulib/calibration/record_store.hpp#L288).*

<a id="savereport-outcome"></a>

### `SaveReport::outcome`

```cpp
SaveResult outcome = SaveResult::NotSaved
```

Saved, NotSaved or Unconfirmed (header: step 4).

*field, declared at [`include/shulib/calibration/record_store.hpp:290`](../../include/shulib/calibration/record_store.hpp#L290).*

<a id="savereport-refused"></a>

### `SaveReport::refused`

```cpp
SaveRefusal refused = SaveRefusal::None
```

Why the save stopped before touching the card, or None.

*field, declared at [`include/shulib/calibration/record_store.hpp:292`](../../include/shulib/calibration/record_store.hpp#L292).*

<a id="savereport-why"></a>

### `SaveReport::why`

```cpp
char why[kFileReasonBytes] = ""
```

The refusal's reason for BytesRefused / DescriptionInvalid / EncodeFailed; "" otherwise.

*field, declared at [`include/shulib/calibration/record_store.hpp:294`](../../include/shulib/calibration/record_store.hpp#L294).*

<a id="savereport-length"></a>

### `SaveReport::length`

```cpp
std::size_t length = 0
```

Length of the bytes this save wrote (or meant to).

*field, declared at [`include/shulib/calibration/record_store.hpp:296`](../../include/shulib/calibration/record_store.hpp#L296).*

<a id="savereport-before"></a>

### `SaveReport::before`

```cpp
Which before = Which::None
```

The record in effect BEFORE the save, by chooseEffective() over step 1's reads.

*field, declared at [`include/shulib/calibration/record_store.hpp:298`](../../include/shulib/calibration/record_store.hpp#L298).*

<a id="savereport-backupfirst"></a>

### `SaveReport::backupFirst`

```cpp
PutRecord backupFirst{}
```

Step 1: the backup made from the primary's old bytes (attempted only when K differed).

*field, declared at [`include/shulib/calibration/record_store.hpp:300`](../../include/shulib/calibration/record_store.hpp#L300).*

<a id="savereport-primarywrite"></a>

### `SaveReport::primaryWrite`

```cpp
PutRecord primaryWrite{}
```

Step 2: the new bytes into the primary.

*field, declared at [`include/shulib/calibration/record_store.hpp:302`](../../include/shulib/calibration/record_store.hpp#L302).*

<a id="savereport-restore"></a>

### `SaveReport::restore`

```cpp
PutRecord restore{}
```

Step 2's restore of the old bytes into the primary (attempted only when needed).

*field, declared at [`include/shulib/calibration/record_store.hpp:304`](../../include/shulib/calibration/record_store.hpp#L304).*

<a id="savereport-backuprefresh"></a>

### `SaveReport::backupRefresh`

```cpp
PutRecord backupRefresh{}
```

Step 3: the new bytes into the backup (attempted only when step 2 verified).

*field, declared at [`include/shulib/calibration/record_store.hpp:306`](../../include/shulib/calibration/record_store.hpp#L306).*

<a id="savereport-unconfirmed"></a>

### `SaveReport::unconfirmed`

```cpp
UnconfirmedWhy unconfirmed = UnconfirmedWhy::None
```

Why the outcome is Unconfirmed, or None.

*field, declared at [`include/shulib/calibration/record_store.hpp:308`](../../include/shulib/calibration/record_store.hpp#L308).*

<a id="savereport-primarydisagreed"></a>

### `SaveReport::primaryDisagreed`

```cpp
bool primaryDisagreed = false
```

On (u3): the primary file did not read back as last seen.

*field, declared at [`include/shulib/calibration/record_store.hpp:310`](../../include/shulib/calibration/record_store.hpp#L310).*

<a id="savereport-backupdisagreed"></a>

### `SaveReport::backupDisagreed`

```cpp
bool backupDisagreed = false
```

On (u3): the backup did not read back as last seen.

*field, declared at [`include/shulib/calibration/record_store.hpp:312`](../../include/shulib/calibration/record_store.hpp#L312).*

<a id="savereport-effective"></a>

### `SaveReport::effective`

```cpp
Which effective = Which::None
```

The file the next boot will use, MEASURED at step 4 — None when Unconfirmed (no claim).

*field, declared at [`include/shulib/calibration/record_store.hpp:314`](../../include/shulib/calibration/record_store.hpp#L314).*

<a id="savereport-primaryafter"></a>

### `SaveReport::primaryAfter`

```cpp
FileHolds primaryAfter = FileHolds::NotMeasured
```

What step 4's read of the primary found (a record of the read, not a claim, when Unconfirmed).

*field, declared at [`include/shulib/calibration/record_store.hpp:317`](../../include/shulib/calibration/record_store.hpp#L317).*

<a id="savereport-backupafter"></a>

### `SaveReport::backupAfter`

```cpp
FileHolds backupAfter = FileHolds::NotMeasured
```

What step 4's read of the backup found (likewise).

*field, declared at [`include/shulib/calibration/record_store.hpp:319`](../../include/shulib/calibration/record_store.hpp#L319).*

<a id="savereport-primarypath"></a>

### `SaveReport::primaryPath`

```cpp
char primaryPath[hal::kStorePathBytes] = ""
```

The primary file's path as the store named it ("" if the card was never touched).

*field, declared at [`include/shulib/calibration/record_store.hpp:321`](../../include/shulib/calibration/record_store.hpp#L321).*

<a id="savereport-backuppath"></a>

### `SaveReport::backupPath`

```cpp
char backupPath[hal::kStorePathBytes] = ""
```

The backup's path as the store named it ("" if the card was never touched).

*field, declared at [`include/shulib/calibration/record_store.hpp:323`](../../include/shulib/calibration/record_store.hpp#L323).*

<a id="savereport-primaryopenedforwriting"></a>

### `SaveReport::primaryOpenedForWriting`

```cpp
[[nodiscard]] constexpr bool primaryOpenedForWriting() const noexcept
```

Whether the save ever asked for the primary file's truncating open (step 2 or its restore) — the destructive moment (HA-166 (1)).

*function, declared at [`include/shulib/calibration/record_store.hpp:327`](../../include/shulib/calibration/record_store.hpp#L327).*

<a id="safereplace"></a>

## `safeReplace`

```cpp
template <std::size_t MaxBytes, typename Accept> [[nodiscard]] SaveReport safeReplace(hal::IFileStore& store, const char* primary, const char* backup, std::span<const std::byte> bytes, Accept&& accept)
```

THE SAFE SAVE, byte-level and generic (header: the design). Replaces `primary` with `bytes`, designed so that a failure part-way leaves the card holding the file in effect before or the new one — given HA-166 (4) and (5), and short of the two-fault limit (header: THE GUARANTEE) — and returns what it MEASURED. `bytes` must be accepted and at most MaxBytes long (a longer one is a precondition), and the two names must be two files on the card (hal::sameFileOnCard()). Not noexcept: preconditions and the store's own throws propagate.

*free function, declared at [`include/shulib/calibration/record_store.hpp:556`](../../include/shulib/calibration/record_store.hpp#L556).*

<a id="acceptrecord"></a>

## `acceptRecord`

```cpp
[[nodiscard]] inline bool acceptRecord(std::span<const std::byte> bytes, char* why, std::size_t whyCap) noexcept
```

The calibration record's accept(): the bytes DECODE — magic, version, length (exactly kRecordBytes), CRC-32 and valid() — with decodeRecord()'s refusal written into `why`.

*free function, declared at [`include/shulib/calibration/record_store.hpp:578`](../../include/shulib/calibration/record_store.hpp#L578).*

<a id="saverecordsafely"></a>

## `saveRecordSafely`

```cpp
[[nodiscard]] inline SaveReport saveRecordSafely(hal::IFileStore& store, const hal::DrivetrainDescription& description)
```

Save a drivetrain description as the calibration record, SAFELY (safeReplace() over kRecordFileName and kRecordBackupFileName). Refuses an invalid description, or one that will not encode, BEFORE touching the card (the report's `refused` and `why` say which).

*free function, declared at [`include/shulib/calibration/record_store.hpp:587`](../../include/shulib/calibration/record_store.hpp#L587).*

<a id="struct-recordfileread"></a>

## `struct RecordFileRead`

```cpp
struct RecordFileRead
```

One record file, as the load saw it: FileRead with `accepted` read as "decoded".

*struct, declared at [`include/shulib/calibration/record_store.hpp:609`](../../include/shulib/calibration/record_store.hpp#L609).*

<a id="recordfileread-tried"></a>

### `RecordFileRead::tried`

```cpp
bool tried = false
```

The load read this file (the backup only when the record file did not decode).

*field, declared at [`include/shulib/calibration/record_store.hpp:611`](../../include/shulib/calibration/record_store.hpp#L611).*

<a id="recordfileread-opened"></a>

### `RecordFileRead::opened`

```cpp
bool opened = false
```

False: no card, or no such file.

*field, declared at [`include/shulib/calibration/record_store.hpp:613`](../../include/shulib/calibration/record_store.hpp#L613).*

<a id="recordfileread-count"></a>

### `RecordFileRead::count`

```cpp
std::size_t count = 0
```

Bytes read; 0 on an open but EMPTY file, and 0 when the read did not open.

*field, declared at [`include/shulib/calibration/record_store.hpp:615`](../../include/shulib/calibration/record_store.hpp#L615).*

<a id="recordfileread-decoded"></a>

### `RecordFileRead::decoded`

```cpp
bool decoded = false
```

decodeRecord() accepted it.

*field, declared at [`include/shulib/calibration/record_store.hpp:617`](../../include/shulib/calibration/record_store.hpp#L617).*

<a id="recordfileread-why"></a>

### `RecordFileRead::why`

```cpp
char why[kFileReasonBytes] = ""
```

decodeRecord()'s refusal, when the file opened, held bytes and did not decode; "" otherwise.

*field, declared at [`include/shulib/calibration/record_store.hpp:619`](../../include/shulib/calibration/record_store.hpp#L619).*

<a id="recordfileread-path"></a>

### `RecordFileRead::path`

```cpp
char path[hal::kStorePathBytes] = ""
```

The path the store's read used (the adapter's path() on the brain), for messages only.

*field, declared at [`include/shulib/calibration/record_store.hpp:621`](../../include/shulib/calibration/record_store.hpp#L621).*

<a id="struct-loadedrecord"></a>

## `struct LoadedRecord`

```cpp
struct LoadedRecord
```

The calibration record as the load found it: which file, the description, and each file's read — enough for a boot to say, per file, "could not be opened", "is EMPTY" or the decoder's reason.

*struct, declared at [`include/shulib/calibration/record_store.hpp:626`](../../include/shulib/calibration/record_store.hpp#L626).*

<a id="loadedrecord-from"></a>

### `LoadedRecord::from`

```cpp
Which from = Which::None
```

Primary, Backup or None.

*field, declared at [`include/shulib/calibration/record_store.hpp:628`](../../include/shulib/calibration/record_store.hpp#L628).*

<a id="loadedrecord-description"></a>

### `LoadedRecord::description`

```cpp
hal::DrivetrainDescription description{}
```

The decoded description; meaningful unless `from` is None.

*field, declared at [`include/shulib/calibration/record_store.hpp:630`](../../include/shulib/calibration/record_store.hpp#L630).*

<a id="loadedrecord-primary"></a>

### `LoadedRecord::primary`

```cpp
RecordFileRead primary{}
```

The record file (kRecordFileName), always read.

*field, declared at [`include/shulib/calibration/record_store.hpp:632`](../../include/shulib/calibration/record_store.hpp#L632).*

<a id="loadedrecord-backup"></a>

### `LoadedRecord::backup`

```cpp
RecordFileRead backup{}
```

The backup (kRecordBackupFileName), read only when the record file did not decode.

*field, declared at [`include/shulib/calibration/record_store.hpp:634`](../../include/shulib/calibration/record_store.hpp#L634).*

<a id="loadrecord"></a>

## `loadRecord`

```cpp
[[nodiscard]] inline LoadedRecord loadRecord(hal::IFileStore& store)
```

Load the calibration record: the record file if it decodes, else its backup if THAT decodes, else none (loadWithBackup() over acceptRecord). The backup is not read when the record file decodes. Not noexcept: the store's own throws propagate.

*free function, declared at [`include/shulib/calibration/record_store.hpp:651`](../../include/shulib/calibration/record_store.hpp#L651).*

<a id="enum-class-bootreadsbackup"></a>

## `enum class BootReadsBackup`

```cpp
enum class BootReadsBackup : std::uint8_t
```

Whether the boot of the driver programs this Calibrate ships with ("shulib Drive" and the competition program since T4 Part 3; "shulib Teleop" before it) reads the BACKUP when the record file does not decode. The save's words depend on it; no default, so every caller states it.

*enum class, declared at [`include/shulib/calibration/record_store.hpp:672`](../../include/shulib/calibration/record_store.hpp#L672).*

<a id="bootreadsbackup-no"></a>

### `BootReadsBackup::No`

```cpp
No
```

the boot reads the record file only: a damaged one REFUSES

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:673`](../../include/shulib/calibration/record_store.hpp#L673).*

<a id="bootreadsbackup-yes"></a>

### `BootReadsBackup::Yes`

```cpp
Yes
```

the boot falls back to a decodable backup (in amber)

*enumerator, declared at [`include/shulib/calibration/record_store.hpp:674`](../../include/shulib/calibration/record_store.hpp#L674).*

<a id="ksavereportlinebytes"></a>

## `kSaveReportLineBytes`

```cpp
inline constexpr std::size_t kSaveReportLineBytes = 240
```

Longest line formatSaveReport() emits, terminator included — exactly Calibrate's emitf buffer, so a line passed through emitf("%s") is never cut. Every format below is budgeted against it with each bounded string at its full precision (what -Wformat-truncation assumes).

*constant, declared at [`include/shulib/calibration/record_store.hpp:680`](../../include/shulib/calibration/record_store.hpp#L680).*

<a id="formatsavereport"></a>

## `formatSaveReport`

```cpp
template <typename Emit> void formatSaveReport(const SaveReport& r, BootReadsBackup bootReadsBackup, Emit&& emit)
```

The serial lines for a save of the DRIVETRAIN record, one `emit(const char*)` per line: ASCII, no '%', each under kSaveReportLineBytes. EVERY sentence is derived from the measured report — "is still on the card" only when the final read found the earlier record in the record file and the next boot uses it (untouched, or written back and re-read). `bootReadsBackup` says whether the boot this build ships with falls back to the backup. The formatter is TOTAL: a measured combination with no named branch prints the measurement itself (which file the next boot uses, and what it holds). The words name the drivetrain record, "shulib Calibrate" and "the next boot" — never a drive program by name: more than one program boots from the record ("shulib Drive", the competition program and the Auton Tester since T4 Part 3, which retired the "shulib Teleop" these words were first written beside). A sibling file words its own lines from the same SaveReport.

*free function, declared at [`include/shulib/calibration/record_store.hpp:745`](../../include/shulib/calibration/record_store.hpp#L745).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 99 lines, click to expand</summary>

```text

 The calibration record's SAFE SAVE and its LOAD (chunk SAFE-SAVE Part A, 2026-09-29): a save
 designed so a failure part-way does not leave the card without a good record — given HA-166 (4)
 and (5), short of a stated two-fault limit — a load that falls back to a verified backup copy,
 and a report that says what the card MEASURABLY holds afterwards.

 ── The defect this exists for (found 2026-09-26) ───────────────────────────────────
 "shulib Calibrate" saved by opening the record file with fopen "wb" and writing it. On this brain
 "wb" reaches VEXos's create-a-file call, which is taken to empty an existing file AT THE OPEN
 (HA-166 (1); reasoned, not measured), so every failure after the open — a refused write, a
 refused flush, a close that cannot report, a pulled card, a power cut — left the card with NO
 usable record, and the program then printed that the earlier record "is still on the card". The
 obvious fix, "write a temporary file and rename it over the old one", cannot be built: the brain
 has no rename and no delete (HA-166 (2)), and no read-write open (HA-166 (3)). A rename-based
 save would pass every host test (Linux has rename) and fail on the robot.

 ── The design: BACKUP FIRST, then the record, then refresh the backup ──────────────
 P is the primary file (the record), K its backup. put(X, b) means: replace(X, b), keep the step
 it reported for the WORDS only, then ALWAYS read X back; put succeeds if and only if the read
 opened and returned exactly b (same length, same bytes) and accept() takes it. The decision
 never uses the step replace() reported — the close cannot report (hal/file_store.hpp), and a
 false flush that kept the bytes is a SAVE — Calibrate's ruling P5 applied as written: "SAVED is
 a measurement or it is not said", and so is its absence.
   0. The bytes must be accepted and a card present; otherwise NOT SAVED with nothing opened.
   1. READ THE STARTING CARD: P and K, both, always — and P a SECOND time, before K, when its
      first read is not accepted. A refused, short or altered read is not a measurement (below:
      "last seen"), and without the re-read one such misread of a GOOD record file would make the
      save treat it as damaged, back nothing up, and truncate the only copy at step 2. The record
      IN EFFECT BEFORE is the file chooseEffective() names. If it is P and K is not byte-identical
      to it, put(K, old) — and if that fails, stop with P never opened for writing. If P is NOT
      accepted, K is not written: it may hold the only good record.
   2. put(P, bytes). If that fails and the old record came from P: if P already reads back as the
      old bytes, nothing changed and there is no restore; otherwise put(P, old) — the restore.
   3. Only if step 2 succeeded: put(K, bytes), refreshing the backup. Its failure changes nothing
      about P.
   4. MEASURE, never assume: read P and K again. The result is UNCONFIRMED, and nothing more is
      claimed, when (u1) the card is gone, (u2) no operation in the whole save got a file open,
      or (u3) a file does not read back as the save last saw it. Otherwise chooseEffective() names
      the file the next boot will use; SAVED if and only if that file is P and it holds the new
      bytes.
 THE GUARANTEE, and its conditions. Given HA-166 (4) (writing one file never damages another) and
 (5) (a file flushed, closed and read back whole is on the medium), ONE failure at any step — a
 refused open or read, a short write or read, a false flush, a write that silently did not land,
 a power cut or a pulled card — leaves the card holding the record in effect before the save, or
 the new one; so does a silent write fault followed by a crash, and a misread the checks can see
 (refused, short, or failing the CRC) at any one read followed by a crash or by a write fault.
 The tests enumerate each of these. THE LIMIT — what it does NOT defend: a misread at step 1 that
 the checks cannot catch, followed by a second failure (a failed write or a power cut) in the
 same save. Two kinds: (i) the record file misread as damaged at BOTH of its first two reads;
 (ii) a read that returns a VALID record the file does not hold — a stale read (HA-139 is the
 belief that a read returns what the card holds): the record file read as some other record
 than it holds (it is damaged, missing or different), or the backup (damaged, older or missing)
 read as identical to the record. The second failure can then lose the record, or leave the next
 boot a record other than the one in effect (an older one). The save's words rest on what step
 1 READ: after (i) they say only that the record file "did not read as usable"; after (ii) they
 call the record the stale read returned "the earlier record", and can say it is still on the
 card. The tests pin both kinds (THE STATED LIMIT, measured).

 ── "Last seen", and why a refused read is not a measurement of absence ─────────────
 For each file the save keeps what it last read — (opened, length, CRC-32), never a copy. A read
 that opens sets it. A read that does NOT open, after a replace of that file got past its open,
 sets it to UNKNOWN: the save has just written that file and there is no delete, so the refusal
 is a failed read, not an absent file. Any other refused read leaves it as it was. Step 4 compares
 against it — which is what keeps a card that fails at the save's very first read from being
 "measured" as empty, and reported as holding nothing, with a good record on it.

 ── The load rule, in one place ─────────────────────────────────────────────────────
 chooseEffective(): P if accepted, else K if accepted, else none. The save's step 1 and step 4 use
 it, and so does loadWithBackup(), which reads K only when P is not accepted. For the record,
 "accepted" is decodeRecord() succeeding — magic, version, length (exactly 180), CRC and valid() —
 so a TORN file is a refusal, never a misread.

 ── Contract ────────────────────────────────────────────────────────────────────────
   * Allocates nothing. Exception-NEUTRAL, never noexcept: it catches nothing and throws only what
     the store throws plus SHULIB_PRECONDITION on its own arguments (null names; two names FAT's
     rule makes ONE file — equal, or differing only in letter case or in trailing dots and
     spaces, hal::sameFileOnCard(), HA-166 (6); bytes longer than MaxBytes). A noexcept here
     would turn an adapter's bare-name precondition into std::terminate with no message.
   * accept is a template parameter (a callable), never std::function:
       bool accept(std::span<const std::byte> bytes, char* why, std::size_t whyCap)
     It sees every byte read, up to MaxBytes + 32 — so a LONGER file reaches it and must be
     refused by length. `why` may be null with whyCap 0. An EMPTY file is never passed to it:
     an empty file is never accepted, and neither are empty bytes to save.
   * A store's read COUNT is never trusted past the buffer it was handed: a count above it (a
     broken store — IFileStore promises "up to into.size()") is clamped to the buffer, so it can
     never become a CRC, a copy or an accept() over bytes past the end of a stack array. And a
     read that did NOT open counts 0 bytes, whatever count it carries — never accepted, never
     copied, never a CRC (both the load and the save check `opened` themselves as well).
   * STACK (sizes measured with the V5's arm-none-eabi-g++, 2026-09-29; re-measured 2026-10-01).
     safeReplace<MaxBytes> holds exactly two arrays of MaxBytes + 32 bytes (the old record, for
     the restore; one read scratch every read reuses) — 424 bytes for the record, 496 with the
     save's other state — and returns a SaveReport (452 bytes) into the caller's frame.
     loadWithBackup<MaxBytes> returns a BackupLoad (MaxBytes + 640: 820 bytes for the record)
     into the caller's frame; loadRecord() holds one of those on its own frame and returns a
     LoadedRecord (792 bytes). MaxBytes is capped at 1024 (kMaxStoredFileBytes) so no caller puts
     a large file on a task stack by accident.

 NO HARDWARE CLAIM: nothing here has run on a brain. The six beliefs it rests on are HA-166.
 PROS-free, sim-free, allocation-free.
```

</details>
