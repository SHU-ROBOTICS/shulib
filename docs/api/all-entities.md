<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Regenerate: python3 tools/api_doc_tool.py generate -->

# Every public entity, alphabetically

All 1,625 of them, across 115 shipped headers: types, their members, nested types and their members, free functions, namespace-scope constants and type aliases. Generated from the headers by the same parse that produces the pages, so a name missing here is a name missing everywhere — which is why the build fails if this file is not byte-identical to a fresh run.

Nested types appear under their qualified name (`BlackboxReader::Frame::type`), so a member of a nested type is findable by the name you would actually write. Overloads are numbered in source order and each has its own link.

The [reference overview](README.md) says what is deliberately *not* here, and why.

[A](#a) · [B](#b) · [C](#c) · [D](#d) · [E](#e) · [F](#f) · [G](#g) · [H](#h) · [I](#i) · [K](#k) · [L](#l) · [M](#m) · [N](#n) · [O](#o) · [P](#p) · [Q](#q) · [R](#r) · [S](#s) · [T](#t) · [V](#v) · [W](#w) · [X](#x) · [Y](#y)

## A

| Name | Kind | Page |
|---|---|---|
| `Acceleration` | type alias | [quantity.md](quantity.md#acceleration) |
| `ActuateAndConfirm` | class | [mechanism_op.md](mechanism_op.md#class-actuateandconfirm) |
| `ActuateAndConfirm::ActuateAndConfirm` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-actuateandconfirm) |
| `ActuateAndConfirm::ActuateAndConfirm (overload 2)` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-actuateandconfirm-2) |
| `ActuateAndConfirm::ActuateAndConfirm (overload 3)` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-actuateandconfirm-3) |
| `ActuateAndConfirm::cancel` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-cancel) |
| `ActuateAndConfirm::name` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-name) |
| `ActuateAndConfirm::operator=` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-operator-eq) |
| `ActuateAndConfirm::operator= (overload 2)` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-operator-eq-2) |
| `ActuateAndConfirm::outcome` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-outcome) |
| `ActuateAndConfirm::start` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-start) |
| `ActuateAndConfirm::started` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-started) |
| `ActuateAndConfirm::tick` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-tick) |
| `ActuateAndConfirm::~ActuateAndConfirm` | function | [mechanism_op.md](mechanism_op.md#actuateandconfirm-destructor-actuateandconfirm) |
| `ActuateAndConfirmConfig` | struct | [mechanism_op.md](mechanism_op.md#struct-actuateandconfirmconfig) |
| `ActuateAndConfirmConfig::actuationTime` | field | [mechanism_op.md](mechanism_op.md#actuateandconfirmconfig-actuationtime) |
| `ActuateAndConfirmConfig::confirmWindow` | field | [mechanism_op.md](mechanism_op.md#actuateandconfirmconfig-confirmwindow) |
| `ActuateAndConfirmConfig::target` | field | [mechanism_op.md](mechanism_op.md#actuateandconfirmconfig-target) |
| `AlwaysConfirmed` | struct | [mechanism_op.md](mechanism_op.md#struct-alwaysconfirmed) |
| `AlwaysConfirmed::operator()` | function | [mechanism_op.md](mechanism_op.md#alwaysconfirmed-operator-call) |
| `Angle` | class | [angle.md](angle.md#class-angle) |
| `Angle::Angle` | function | [angle.md](angle.md#angle-angle) |
| `Angle::approxEqual` | function | [angle.md](angle.md#angle-approxequal) |
| `Angle::degrees` | function | [angle.md](angle.md#angle-degrees) |
| `Angle::degrees (overload 2)` | function | [angle.md](angle.md#angle-degrees-2) |
| `Angle::errorTo` | function | [angle.md](angle.md#angle-errorto) |
| `Angle::kPi` | field | [angle.md](angle.md#angle-kpi) |
| `Angle::operator+` | function | [angle.md](angle.md#angle-operator-plus) |
| `Angle::operator-` | function | [angle.md](angle.md#angle-operator-minus) |
| `Angle::operator- (overload 2)` | function | [angle.md](angle.md#angle-operator-minus-2) |
| `Angle::radians` | function | [angle.md](angle.md#angle-radians) |
| `Angle::radians (overload 2)` | function | [angle.md](angle.md#angle-radians-2) |
| `AngleDim` | type alias | [quantity.md](quantity.md#angledim) |
| `AngularVelocity` | type alias | [quantity.md](quantity.md#angularvelocity) |
| `appendNum` | free function | [line_format.md](line_format.md#appendnum) |
| `appendPadded` | free function | [line_format.md](line_format.md#appendpadded) |
| `appendTimestamp` | free function | [line_format.md](line_format.md#appendtimestamp) |
| `appendUnsigned` | free function | [line_format.md](line_format.md#appendunsigned) |
| `AppliedCorrection` | struct | [correction.md](correction.md#struct-appliedcorrection) |
| `AppliedCorrection::audit` | field | [correction.md](correction.md#appliedcorrection-audit) |
| `AppliedCorrection::clamped` | field | [correction.md](correction.md#appliedcorrection-clamped) |
| `AppliedCorrection::dtheta` | field | [correction.md](correction.md#appliedcorrection-dtheta) |
| `AppliedCorrection::dx` | field | [correction.md](correction.md#appliedcorrection-dx) |
| `AppliedCorrection::dy` | field | [correction.md](correction.md#appliedcorrection-dy) |
| `AppliedCorrection::gated` | field | [correction.md](correction.md#appliedcorrection-gated) |
| `AppliedCorrection::source` | field | [correction.md](correction.md#appliedcorrection-source) |
| `applyCancelSafeState` | free function | [motion.md](motion.md#applycancelsafestate) |
| `applyCommandPipeline` | free function | [command_pipeline.md](command_pipeline.md#applycommandpipeline) |
| `AprilTagCorrector` | class | [apriltag_corrector.md](apriltag_corrector.md#class-apriltagcorrector) |
| `AprilTagCorrector::acceptedFixes` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-acceptedfixes) |
| `AprilTagCorrector::AprilTagCorrector` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-apriltagcorrector) |
| `AprilTagCorrector::innovationRejects` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-innovationrejects) |
| `AprilTagCorrector::kHistory` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-khistory) |
| `AprilTagCorrector::kMaxTagsPerFrame` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-kmaxtagsperframe) |
| `AprilTagCorrector::kMinConfidenceFloor` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-kminconfidencefloor) |
| `AprilTagCorrector::lastTagId` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-lasttagid) |
| `AprilTagCorrector::lastVerdict` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-lastverdict) |
| `AprilTagCorrector::name` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-name) |
| `AprilTagCorrector::noFrameTicks` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-noframeticks) |
| `AprilTagCorrector::noTagTicks` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-notagticks) |
| `AprilTagCorrector::poll` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-poll) |
| `AprilTagCorrector::pollCount` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-pollcount) |
| `AprilTagCorrector::propose` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-propose) |
| `AprilTagCorrector::qualityRejects` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-qualityrejects) |
| `AprilTagCorrector::rangeRejects` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-rangerejects) |
| `AprilTagCorrector::staleFrameTicks` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-staleframeticks) |
| `AprilTagCorrector::staleTicks` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-staleticks) |
| `AprilTagCorrector::travelSinceFix` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-travelsincefix) |
| `AprilTagCorrector::unmappedRejects` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-unmappedrejects) |
| `AprilTagCorrector::yawRateRejects` | function | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrector-yawraterejects) |
| `AprilTagCorrectorConfig` | struct | [apriltag_corrector.md](apriltag_corrector.md#struct-apriltagcorrectorconfig) |
| `AprilTagCorrectorConfig::baseStdDev` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-basestddev) |
| `AprilTagCorrectorConfig::driftStdDevPerInch` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-driftstddevperinch) |
| `AprilTagCorrectorConfig::gateSigma` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-gatesigma) |
| `AprilTagCorrectorConfig::latency` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-latency) |
| `AprilTagCorrectorConfig::maxObservationAge` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-maxobservationage) |
| `AprilTagCorrectorConfig::maxRange` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-maxrange) |
| `AprilTagCorrectorConfig::maxYawRate` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-maxyawrate) |
| `AprilTagCorrectorConfig::minConfidence` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-minconfidence) |
| `AprilTagCorrectorConfig::minRange` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-minrange) |
| `AprilTagCorrectorConfig::postFixStdDev` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-postfixstddev) |
| `AprilTagCorrectorConfig::stdDevPerInch` | field | [apriltag_corrector.md](apriltag_corrector.md#apriltagcorrectorconfig-stddevperinch) |
| `arcStep` | free function | [arc_step.md](arc_step.md#arcstep) |
| `AxisGains` | struct | [motion_config.md](motion_config.md#struct-axisgains) |
| `AxisGains::integralLimit` | field | [motion_config.md](motion_config.md#axisgains-integrallimit) |
| `AxisGains::kD` | field | [motion_config.md](motion_config.md#axisgains-kd) |
| `AxisGains::kI` | field | [motion_config.md](motion_config.md#axisgains-ki) |
| `AxisGains::kP` | field | [motion_config.md](motion_config.md#axisgains-kp) |

## B

| Name | Kind | Page |
|---|---|---|
| `BlackboxHeader` | struct | [blackbox_format.md](blackbox_format.md#struct-blackboxheader) |
| `BlackboxHeader::alliance` | function | [blackbox_format.md](blackbox_format.md#blackboxheader-alliance) |
| `BlackboxHeader::alliance_` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-alliance_) |
| `BlackboxHeader::buildHash` | function | [blackbox_format.md](blackbox_format.md#blackboxheader-buildhash) |
| `BlackboxHeader::buildHash_` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-buildhash_) |
| `BlackboxHeader::byteBudget` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-bytebudget) |
| `BlackboxHeader::epochSeconds` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-epochseconds) |
| `BlackboxHeader::flags` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-flags) |
| `BlackboxHeader::formatVersion` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-formatversion) |
| `BlackboxHeader::headerBytes` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-headerbytes) |
| `BlackboxHeader::portMap` | function | [blackbox_format.md](blackbox_format.md#blackboxheader-portmap) |
| `BlackboxHeader::portMap_` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-portmap_) |
| `BlackboxHeader::ringCapacity` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-ringcapacity) |
| `BlackboxHeader::routineId` | function | [blackbox_format.md](blackbox_format.md#blackboxheader-routineid) |
| `BlackboxHeader::routineId_` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-routineid_) |
| `BlackboxHeader::side` | function | [blackbox_format.md](blackbox_format.md#blackboxheader-side) |
| `BlackboxHeader::side_` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-side_) |
| `BlackboxHeader::tickRecordBytes` | field | [blackbox_format.md](blackbox_format.md#blackboxheader-tickrecordbytes) |
| `BlackboxReader` | class | [blackbox_reader.md](blackbox_reader.md#class-blackboxreader) |
| `BlackboxReader::BlackboxReader` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-blackboxreader) |
| `BlackboxReader::bytesConsumed` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-bytesconsumed) |
| `BlackboxReader::Frame` | struct | [blackbox_reader.md](blackbox_reader.md#struct-blackboxreader-frame) |
| `BlackboxReader::Frame::payload` | field | [blackbox_reader.md](blackbox_reader.md#blackboxreader-frame-payload) |
| `BlackboxReader::Frame::type` | field | [blackbox_reader.md](blackbox_reader.md#blackboxreader-frame-type) |
| `BlackboxReader::framesRead` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-framesread) |
| `BlackboxReader::header` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-header) |
| `BlackboxReader::next` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-next) |
| `BlackboxReader::sawEnd` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-sawend) |
| `BlackboxReader::skippedFrames` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-skippedframes) |
| `BlackboxReader::status` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-status) |
| `BlackboxReader::truncated` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-truncated) |
| `BlackboxReader::truncatedFrameType` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-truncatedframetype) |
| `BlackboxReader::usable` | function | [blackbox_reader.md](blackbox_reader.md#blackboxreader-usable) |
| `BodyTravel` | struct | [arc_step.md](arc_step.md#struct-bodytravel) |
| `BodyTravel::forward` | field | [arc_step.md](arc_step.md#bodytravel-forward) |
| `BodyTravel::lateral` | field | [arc_step.md](arc_step.md#bodytravel-lateral) |
| `BrakeMode` | enum class | [motor.md](motor.md#enum-class-brakemode) |
| `BrakeMode::Brake` | enumerator | [motor.md](motor.md#brakemode-brake) |
| `BrakeMode::Coast` | enumerator | [motor.md](motor.md#brakemode-coast) |
| `BrakeMode::Hold` | enumerator | [motor.md](motor.md#brakemode-hold) |
| `ButtonEdge` | class | [controller.md](controller.md#class-buttonedge) |
| `ButtonEdge::update` | function | [controller.md](controller.md#buttonedge-update) |
| `ByteReader` | class | [blackbox_format.md](blackbox_format.md#class-bytereader) |
| `ByteReader::boolean` | function | [blackbox_format.md](blackbox_format.md#bytereader-boolean) |
| `ByteReader::ByteReader` | function | [blackbox_format.md](blackbox_format.md#bytereader-bytereader) |
| `ByteReader::f64` | function | [blackbox_format.md](blackbox_format.md#bytereader-f64) |
| `ByteReader::i32` | function | [blackbox_format.md](blackbox_format.md#bytereader-i32) |
| `ByteReader::offset` | function | [blackbox_format.md](blackbox_format.md#bytereader-offset) |
| `ByteReader::ok` | function | [blackbox_format.md](blackbox_format.md#bytereader-ok) |
| `ByteReader::skip` | function | [blackbox_format.md](blackbox_format.md#bytereader-skip) |
| `ByteReader::text` | function | [blackbox_format.md](blackbox_format.md#bytereader-text) |
| `ByteReader::u16` | function | [blackbox_format.md](blackbox_format.md#bytereader-u16) |
| `ByteReader::u32` | function | [blackbox_format.md](blackbox_format.md#bytereader-u32) |
| `ByteReader::u8` | function | [blackbox_format.md](blackbox_format.md#bytereader-u8) |
| `ByteWriter` | class | [blackbox_format.md](blackbox_format.md#class-bytewriter) |
| `ByteWriter::boolean` | function | [blackbox_format.md](blackbox_format.md#bytewriter-boolean) |
| `ByteWriter::ByteWriter` | function | [blackbox_format.md](blackbox_format.md#bytewriter-bytewriter) |
| `ByteWriter::f64` | function | [blackbox_format.md](blackbox_format.md#bytewriter-f64) |
| `ByteWriter::i32` | function | [blackbox_format.md](blackbox_format.md#bytewriter-i32) |
| `ByteWriter::offset` | function | [blackbox_format.md](blackbox_format.md#bytewriter-offset) |
| `ByteWriter::ok` | function | [blackbox_format.md](blackbox_format.md#bytewriter-ok) |
| `ByteWriter::text` | function | [blackbox_format.md](blackbox_format.md#bytewriter-text) |
| `ByteWriter::u16` | function | [blackbox_format.md](blackbox_format.md#bytewriter-u16) |
| `ByteWriter::u32` | function | [blackbox_format.md](blackbox_format.md#bytewriter-u32) |
| `ByteWriter::u8` | function | [blackbox_format.md](blackbox_format.md#bytewriter-u8) |
| `ByteWriter::zeros` | function | [blackbox_format.md](blackbox_format.md#bytewriter-zeros) |

## C

| Name | Kind | Page |
|---|---|---|
| `CameraIntrinsics` | struct | [vision_conversion.md](vision_conversion.md#struct-cameraintrinsics) |
| `CameraIntrinsics::cx` | field | [vision_conversion.md](vision_conversion.md#cameraintrinsics-cx) |
| `CameraIntrinsics::cy` | field | [vision_conversion.md](vision_conversion.md#cameraintrinsics-cy) |
| `CameraIntrinsics::fx` | field | [vision_conversion.md](vision_conversion.md#cameraintrinsics-fx) |
| `CameraIntrinsics::fy` | field | [vision_conversion.md](vision_conversion.md#cameraintrinsics-fy) |
| `CameraMount` | struct | [vision_conversion.md](vision_conversion.md#struct-cameramount) |
| `CameraMount::x` | field | [vision_conversion.md](vision_conversion.md#cameramount-x) |
| `CameraMount::y` | field | [vision_conversion.md](vision_conversion.md#cameramount-y) |
| `CameraMount::yaw` | field | [vision_conversion.md](vision_conversion.md#cameramount-yaw) |
| `Chassis` | class | [chassis.md](chassis.md#class-chassis) |
| `Chassis::brake` | function | [chassis.md](chassis.md#chassis-brake) |
| `Chassis::cancel` | function | [chassis.md](chassis.md#chassis-cancel) |
| `Chassis::Chassis` | function | [chassis.md](chassis.md#chassis-chassis) |
| `Chassis::Chassis (overload 2)` | function | [chassis.md](chassis.md#chassis-chassis-2) |
| `Chassis::Chassis (overload 3)` | function | [chassis.md](chassis.md#chassis-chassis-3) |
| `Chassis::deps` | function | [chassis.md](chassis.md#chassis-deps) |
| `Chassis::drive` | function | [chassis.md](chassis.md#chassis-drive) |
| `Chassis::followTrajectory` | function | [chassis.md](chassis.md#chassis-followtrajectory) |
| `Chassis::followTrajectory (overload 2)` | function | [chassis.md](chassis.md#chassis-followtrajectory-2) |
| `Chassis::hold` | function | [chassis.md](chassis.md#chassis-hold) |
| `Chassis::lastCompleted` | function | [chassis.md](chassis.md#chassis-lastcompleted) |
| `Chassis::lastExitReason` | function | [chassis.md](chassis.md#chassis-lastexitreason) |
| `Chassis::motionConfig` | function | [chassis.md](chassis.md#chassis-motionconfig) |
| `Chassis::moveTo` | function | [chassis.md](chassis.md#chassis-moveto) |
| `Chassis::operator=` | function | [chassis.md](chassis.md#chassis-operator-eq) |
| `Chassis::operator= (overload 2)` | function | [chassis.md](chassis.md#chassis-operator-eq-2) |
| `Chassis::pose` | function | [chassis.md](chassis.md#chassis-pose) |
| `Chassis::scheduler` | function | [chassis.md](chassis.md#chassis-scheduler) |
| `Chassis::scheduler (overload 2)` | function | [chassis.md](chassis.md#chassis-scheduler-2) |
| `Chassis::setPose` | function | [chassis.md](chassis.md#chassis-setpose) |
| `Chassis::strafeAuthority` | function | [chassis.md](chassis.md#chassis-strafeauthority) |
| `Chassis::strafeTo` | function | [chassis.md](chassis.md#chassis-strafeto) |
| `Chassis::turnTo` | function | [chassis.md](chassis.md#chassis-turnto) |
| `Chassis::wait` | function | [chassis.md](chassis.md#chassis-wait) |
| `Chassis::waitUntil` | function | [chassis.md](chassis.md#chassis-waituntil) |
| `Chassis::~Chassis` | function | [chassis.md](chassis.md#chassis-destructor-chassis) |
| `ChassisConfig` | struct | [chassis.md](chassis.md#struct-chassisconfig) |
| `ChassisConfig::motion` | field | [chassis.md](chassis.md#chassisconfig-motion) |
| `ChassisConfig::scheduler` | field | [chassis.md](chassis.md#chassisconfig-scheduler) |
| `ChassisSpeeds` | class | [twist2d.md](twist2d.md#class-chassisspeeds) |
| `ChassisSpeeds::approxEqual` | function | [twist2d.md](twist2d.md#chassisspeeds-approxequal) |
| `ChassisSpeeds::ChassisSpeeds` | function | [twist2d.md](twist2d.md#chassisspeeds-chassisspeeds) |
| `ChassisSpeeds::ChassisSpeeds (overload 2)` | function | [twist2d.md](twist2d.md#chassisspeeds-chassisspeeds-2) |
| `ChassisSpeeds::omega` | function | [twist2d.md](twist2d.md#chassisspeeds-omega) |
| `ChassisSpeeds::vx` | function | [twist2d.md](twist2d.md#chassisspeeds-vx) |
| `ChassisSpeeds::vy` | function | [twist2d.md](twist2d.md#chassisspeeds-vy) |
| `CommandIdStampSink` | class | [motion_scheduler.md](motion_scheduler.md#class-commandidstampsink) |
| `CommandIdStampSink::activeId` | function | [motion_scheduler.md](motion_scheduler.md#commandidstampsink-activeid) |
| `CommandIdStampSink::beginTick` | function | [motion_scheduler.md](motion_scheduler.md#commandidstampsink-begintick) |
| `CommandIdStampSink::CommandIdStampSink` | function | [motion_scheduler.md](motion_scheduler.md#commandidstampsink-commandidstampsink) |
| `CommandIdStampSink::emit` | function | [motion_scheduler.md](motion_scheduler.md#commandidstampsink-emit) |
| `CommandIdStampSink::log` | function | [motion_scheduler.md](motion_scheduler.md#commandidstampsink-log) |
| `CommandIdStampSink::setActiveId` | function | [motion_scheduler.md](motion_scheduler.md#commandidstampsink-setactiveid) |
| `CommandIdStampSink::setEstimatorAudit` | function | [motion_scheduler.md](motion_scheduler.md#commandidstampsink-setestimatoraudit) |
| `CommandIdStampSink::setTickPhases` | function | [motion_scheduler.md](motion_scheduler.md#commandidstampsink-settickphases) |
| `CommandIdStampSink::summarize` | function | [motion_scheduler.md](motion_scheduler.md#commandidstampsink-summarize) |
| `CommandIdStampSink::wantsRecord` | function | [motion_scheduler.md](motion_scheduler.md#commandidstampsink-wantsrecord) |
| `CommandOutcome` | struct | [command_pipeline.md](command_pipeline.md#struct-commandoutcome) |
| `CommandOutcome::body` | field | [command_pipeline.md](command_pipeline.md#commandoutcome-body) |
| `CommandOutcome::strafeFallback` | field | [command_pipeline.md](command_pipeline.md#commandoutcome-strafefallback) |
| `commandWithinCapability` | free function | [plausibility_guard.md](plausibility_guard.md#commandwithincapability) |
| `CompensatedVoltage` | struct | [feedforward.md](feedforward.md#struct-compensatedvoltage) |
| `CompensatedVoltage::brownoutLimited` | field | [feedforward.md](feedforward.md#compensatedvoltage-brownoutlimited) |
| `CompensatedVoltage::voltage` | field | [feedforward.md](feedforward.md#compensatedvoltage-voltage) |
| `compensateForBattery` | free function | [feedforward.md](feedforward.md#compensateforbattery) |
| `compiledBuildHash` | free function | [build_info.md](build_info.md#compiledbuildhash) |
| `ComplementaryFusion` | class | [complementary_fusion.md](complementary_fusion.md#class-complementaryfusion) |
| `ComplementaryFusion::ComplementaryFusion` | function | [complementary_fusion.md](complementary_fusion.md#complementaryfusion-complementaryfusion) |
| `ComplementaryFusion::fuse` | function | [complementary_fusion.md](complementary_fusion.md#complementaryfusion-fuse) |
| `ComplementaryFusionConfig` | struct | [complementary_fusion.md](complementary_fusion.md#struct-complementaryfusionconfig) |
| `ComplementaryFusionConfig::headingGate` | field | [complementary_fusion.md](complementary_fusion.md#complementaryfusionconfig-headinggate) |
| `ComplementaryFusionConfig::innovationGate` | field | [complementary_fusion.md](complementary_fusion.md#complementaryfusionconfig-innovationgate) |
| `ComplementaryFusionConfig::maxGain` | field | [complementary_fusion.md](complementary_fusion.md#complementaryfusionconfig-maxgain) |
| `ComplementaryFusionConfig::maxHeadingGain` | field | [complementary_fusion.md](complementary_fusion.md#complementaryfusionconfig-maxheadinggain) |
| `ComplementaryFusionConfig::maxHeadingNudgeRate` | field | [complementary_fusion.md](complementary_fusion.md#complementaryfusionconfig-maxheadingnudgerate) |
| `ComplementaryFusionConfig::maxNudgeRate` | field | [complementary_fusion.md](complementary_fusion.md#complementaryfusionconfig-maxnudgerate) |
| `CompletedMotion` | struct | [motion_scheduler.md](motion_scheduler.md#struct-completedmotion) |
| `CompletedMotion::abortFault` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-abortfault) |
| `CompletedMotion::drift` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-drift) |
| `CompletedMotion::endTime` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-endtime) |
| `CompletedMotion::exit` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-exit) |
| `CompletedMotion::finalPose` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-finalpose) |
| `CompletedMotion::hasPathData` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-haspathdata) |
| `CompletedMotion::id` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-id) |
| `CompletedMotion::name` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-name) |
| `CompletedMotion::overshoot` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-overshoot) |
| `CompletedMotion::preempted` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-preempted) |
| `CompletedMotion::startTime` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-starttime) |
| `CompletedMotion::targetPose` | field | [motion_scheduler.md](motion_scheduler.md#completedmotion-targetpose) |
| `ControllerAxis` | enum class | [controller.md](controller.md#enum-class-controlleraxis) |
| `ControllerAxis::LeftX` | enumerator | [controller.md](controller.md#controlleraxis-leftx) |
| `ControllerAxis::LeftY` | enumerator | [controller.md](controller.md#controlleraxis-lefty) |
| `ControllerAxis::RightX` | enumerator | [controller.md](controller.md#controlleraxis-rightx) |
| `ControllerAxis::RightY` | enumerator | [controller.md](controller.md#controlleraxis-righty) |
| `controllerAxisToCanonical` | free function | [controller_conversion.md](controller_conversion.md#controlleraxistocanonical) |
| `ControllerButton` | enum class | [controller.md](controller.md#enum-class-controllerbutton) |
| `ControllerButton::A` | enumerator | [controller.md](controller.md#controllerbutton-a) |
| `ControllerButton::B` | enumerator | [controller.md](controller.md#controllerbutton-b) |
| `ControllerButton::Down` | enumerator | [controller.md](controller.md#controllerbutton-down) |
| `ControllerButton::L1` | enumerator | [controller.md](controller.md#controllerbutton-l1) |
| `ControllerButton::L2` | enumerator | [controller.md](controller.md#controllerbutton-l2) |
| `ControllerButton::Left` | enumerator | [controller.md](controller.md#controllerbutton-left) |
| `ControllerButton::R1` | enumerator | [controller.md](controller.md#controllerbutton-r1) |
| `ControllerButton::R2` | enumerator | [controller.md](controller.md#controllerbutton-r2) |
| `ControllerButton::Right` | enumerator | [controller.md](controller.md#controllerbutton-right) |
| `ControllerButton::Up` | enumerator | [controller.md](controller.md#controllerbutton-up) |
| `ControllerButton::X` | enumerator | [controller.md](controller.md#controllerbutton-x) |
| `ControllerButton::Y` | enumerator | [controller.md](controller.md#controllerbutton-y) |
| `ControllerFaultDisplay` | class | [controller_display.md](controller_display.md#class-controllerfaultdisplay) |
| `ControllerFaultDisplay::ControllerFaultDisplay` | function | [controller_display.md](controller_display.md#controllerfaultdisplay-controllerfaultdisplay) |
| `ControllerFaultDisplay::update` | function | [controller_display.md](controller_display.md#controllerfaultdisplay-update) |
| `ControllerId` | enum class | [pros-controller.md](pros-controller.md#enum-class-controllerid) |
| `ControllerId::Master` | enumerator | [pros-controller.md](pros-controller.md#controllerid-master) |
| `ControllerId::Partner` | enumerator | [pros-controller.md](pros-controller.md#controllerid-partner) |
| `CorrectionProposal` | struct | [correction.md](correction.md#struct-correctionproposal) |
| `CorrectionProposal::confidence` | field | [correction.md](correction.md#correctionproposal-confidence) |
| `CorrectionProposal::fieldPose` | field | [correction.md](correction.md#correctionproposal-fieldpose) |
| `CorrectionProposal::positionStdDev` | field | [correction.md](correction.md#correctionproposal-positionstddev) |
| `CorrectionProposal::providesHeading` | field | [correction.md](correction.md#correctionproposal-providesheading) |
| `CorrectionProposal::selfAudit` | field | [correction.md](correction.md#correctionproposal-selfaudit) |
| `CorrectionProposal::valid` | field | [correction.md](correction.md#correctionproposal-valid) |
| `Current` | type alias | [quantity.md](quantity.md#current) |

## D

| Name | Kind | Page |
|---|---|---|
| `DebugRecord` | struct | [debug_record.md](debug_record.md#struct-debugrecord) |
| `DebugRecord::activeCommandId` | field | [debug_record.md](debug_record.md#debugrecord-activecommandid) |
| `DebugRecord::activeCommandState` | field | [debug_record.md](debug_record.md#debugrecord-activecommandstate) |
| `DebugRecord::batteryCurrent` | field | [debug_record.md](debug_record.md#debugrecord-batterycurrent) |
| `DebugRecord::batteryVoltage` | field | [debug_record.md](debug_record.md#debugrecord-batteryvoltage) |
| `DebugRecord::clampedThisTick` | field | [debug_record.md](debug_record.md#debugrecord-clampedthistick) |
| `DebugRecord::commanded` | field | [debug_record.md](debug_record.md#debugrecord-commanded) |
| `DebugRecord::correctionDTheta` | field | [debug_record.md](debug_record.md#debugrecord-correctiondtheta) |
| `DebugRecord::correctionDx` | field | [debug_record.md](debug_record.md#debugrecord-correctiondx) |
| `DebugRecord::correctionDy` | field | [debug_record.md](debug_record.md#debugrecord-correctiondy) |
| `DebugRecord::covarianceTrace` | field | [debug_record.md](debug_record.md#debugrecord-covariancetrace) |
| `DebugRecord::deadReckoning` | field | [debug_record.md](debug_record.md#debugrecord-deadreckoning) |
| `DebugRecord::droppedLines` | field | [debug_record.md](debug_record.md#debugrecord-droppedlines) |
| `DebugRecord::droppedRecords` | field | [debug_record.md](debug_record.md#debugrecord-droppedrecords) |
| `DebugRecord::dt` | field | [debug_record.md](debug_record.md#debugrecord-dt) |
| `DebugRecord::errorHeading` | field | [debug_record.md](debug_record.md#debugrecord-errorheading) |
| `DebugRecord::errorX` | field | [debug_record.md](debug_record.md#debugrecord-errorx) |
| `DebugRecord::errorY` | field | [debug_record.md](debug_record.md#debugrecord-errory) |
| `DebugRecord::fault` | field | [debug_record.md](debug_record.md#debugrecord-fault) |
| `DebugRecord::gateMahalanobis` | field | [debug_record.md](debug_record.md#debugrecord-gatemahalanobis) |
| `DebugRecord::gateReason` | field | [debug_record.md](debug_record.md#debugrecord-gatereason) |
| `DebugRecord::gateResidualHeading` | field | [debug_record.md](debug_record.md#debugrecord-gateresidualheading) |
| `DebugRecord::gateResidualX` | field | [debug_record.md](debug_record.md#debugrecord-gateresidualx) |
| `DebugRecord::gateResidualY` | field | [debug_record.md](debug_record.md#debugrecord-gateresidualy) |
| `DebugRecord::imuYaw` | field | [debug_record.md](debug_record.md#debugrecord-imuyaw) |
| `DebugRecord::imuYawRate` | field | [debug_record.md](debug_record.md#debugrecord-imuyawrate) |
| `DebugRecord::kMaxWheels` | field | [debug_record.md](debug_record.md#debugrecord-kmaxwheels) |
| `DebugRecord::measuredPose` | field | [debug_record.md](debug_record.md#debugrecord-measuredpose) |
| `DebugRecord::quality` | field | [debug_record.md](debug_record.md#debugrecord-quality) |
| `DebugRecord::qualityClass` | field | [debug_record.md](debug_record.md#debugrecord-qualityclass) |
| `DebugRecord::strafeFallbackActive` | field | [debug_record.md](debug_record.md#debugrecord-strafefallbackactive) |
| `DebugRecord::t` | field | [debug_record.md](debug_record.md#debugrecord-t) |
| `DebugRecord::targetPose` | field | [debug_record.md](debug_record.md#debugrecord-targetpose) |
| `DebugRecord::tickPhase` | field | [debug_record.md](debug_record.md#debugrecord-tickphase) |
| `DebugRecord::wheelCount` | field | [debug_record.md](debug_record.md#debugrecord-wheelcount) |
| `DebugRecord::wheelCurrent` | field | [debug_record.md](debug_record.md#debugrecord-wheelcurrent) |
| `DebugRecord::wheelVoltage` | field | [debug_record.md](debug_record.md#debugrecord-wheelvoltage) |
| `decodeEnd` | free function | [blackbox_format.md](blackbox_format.md#decodeend) |
| `decodeHeader` | free function | [blackbox_format.md](blackbox_format.md#decodeheader) |
| `decodeSummary` | free function | [blackbox_format.md](blackbox_format.md#decodesummary) |
| `decodeTick` | free function | [blackbox_format.md](blackbox_format.md#decodetick) |
| `decodeTriage` | free function | [blackbox_format.md](blackbox_format.md#decodetriage) |
| `desaturateUniform` | free function | [desaturate.md](desaturate.md#desaturateuniform) |
| `DisplayController` | enum class | [pros-line_display.md](pros-line_display.md#enum-class-displaycontroller) |
| `DisplayController::Master` | enumerator | [pros-line_display.md](pros-line_display.md#displaycontroller-master) |
| `DisplayController::Partner` | enumerator | [pros-line_display.md](pros-line_display.md#displaycontroller-partner) |
| `distanceConfidenceToCanonical` | free function | [distance_conversion.md](distance_conversion.md#distanceconfidencetocanonical) |
| `distanceMmToCanonical` | free function | [distance_conversion.md](distance_conversion.md#distancemmtocanonical) |
| `DriveBrake` | class | [drive_brake.md](drive_brake.md#class-drivebrake) |
| `DriveBrake::cancel` | function | [drive_brake.md](drive_brake.md#drivebrake-cancel) |
| `DriveBrake::DriveBrake` | function | [drive_brake.md](drive_brake.md#drivebrake-drivebrake) |
| `DriveBrake::exitReason` | function | [drive_brake.md](drive_brake.md#drivebrake-exitreason) |
| `DriveBrake::kTwistAvgTicks` | field | [drive_brake.md](drive_brake.md#drivebrake-ktwistavgticks) |
| `DriveBrake::name` | function | [drive_brake.md](drive_brake.md#drivebrake-name) |
| `DriveBrake::start` | function | [drive_brake.md](drive_brake.md#drivebrake-start) |
| `DriveBrake::state` | function | [drive_brake.md](drive_brake.md#drivebrake-state) |
| `DriveBrake::tick` | function | [drive_brake.md](drive_brake.md#drivebrake-tick) |

## E

| Name | Kind | Page |
|---|---|---|
| `EkfFusion` | class | [ekf_fusion.md](ekf_fusion.md#class-ekffusion) |
| `EkfFusion::acceptedFixes` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-acceptedfixes) |
| `EkfFusion::consecutiveRejects` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-consecutiverejects) |
| `EkfFusion::covariance` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-covariance) |
| `EkfFusion::EkfFusion` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-ekffusion) |
| `EkfFusion::everReinit` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-everreinit) |
| `EkfFusion::fuse` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-fuse) |
| `EkfFusion::kN` | field | [ekf_fusion.md](ekf_fusion.md#ekffusion-kn) |
| `EkfFusion::kPx` | field | [ekf_fusion.md](ekf_fusion.md#ekffusion-kpx) |
| `EkfFusion::kPy` | field | [ekf_fusion.md](ekf_fusion.md#ekffusion-kpy) |
| `EkfFusion::kTh` | field | [ekf_fusion.md](ekf_fusion.md#ekffusion-kth) |
| `EkfFusion::kVx` | field | [ekf_fusion.md](ekf_fusion.md#ekffusion-kvx) |
| `EkfFusion::kVy` | field | [ekf_fusion.md](ekf_fusion.md#ekffusion-kvy) |
| `EkfFusion::lastCorrectionMagnitude` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-lastcorrectionmagnitude) |
| `EkfFusion::lastHeadingCorrectionMagnitude` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-lastheadingcorrectionmagnitude) |
| `EkfFusion::numericGuardTrips` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-numericguardtrips) |
| `EkfFusion::positionCovarianceTrace` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-positioncovariancetrace) |
| `EkfFusion::reinitCount` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-reinitcount) |
| `EkfFusion::rejectedFixes` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-rejectedfixes) |
| `EkfFusion::resyncCount` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-resynccount) |
| `EkfFusion::state` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-state) |
| `EkfFusion::velocityX` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-velocityx) |
| `EkfFusion::velocityY` | function | [ekf_fusion.md](ekf_fusion.md#ekffusion-velocityy) |
| `EkfFusionConfig` | struct | [ekf_fusion.md](ekf_fusion.md#struct-ekffusionconfig) |
| `EkfFusionConfig::gateSigma` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-gatesigma) |
| `EkfFusionConfig::headingDriftRate` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-headingdriftrate) |
| `EkfFusionConfig::headingNoisePerRad` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-headingnoiseperrad) |
| `EkfFusionConfig::headingStdDev` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-headingstddev) |
| `EkfFusionConfig::initialHeadingStdDev` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-initialheadingstddev) |
| `EkfFusionConfig::initialPosStdDev` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-initialposstddev) |
| `EkfFusionConfig::initialVelStdDev` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-initialvelstddev) |
| `EkfFusionConfig::maxDt` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-maxdt) |
| `EkfFusionConfig::maxHeadingNudgeRate` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-maxheadingnudgerate) |
| `EkfFusionConfig::maxNudgeRate` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-maxnudgerate) |
| `EkfFusionConfig::odomStdDev` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-odomstddev) |
| `EkfFusionConfig::odomStdDevPerInch` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-odomstddevperinch) |
| `EkfFusionConfig::posNoisePerInch` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-posnoiseperinch) |
| `EkfFusionConfig::posNoiseRate` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-posnoiserate) |
| `EkfFusionConfig::reinitCooldown` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-reinitcooldown) |
| `EkfFusionConfig::reinitInnovation` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-reinitinnovation) |
| `EkfFusionConfig::reinitRejectCount` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-reinitrejectcount) |
| `EkfFusionConfig::velNoise` | field | [ekf_fusion.md](ekf_fusion.md#ekffusionconfig-velnoise) |
| `emitRecord` | free function | [telemetry_sink.md](telemetry_sink.md#emitrecord) |
| `emitResultLine` | free function | [motion_result.md](motion_result.md#emitresultline) |
| `emitSessionHeader` | free function | [session_info.md](session_info.md#emitsessionheader) |
| `emitTriageBlock` | free function | [triage.md](triage.md#emittriageblock) |
| `encodeEnd` | free function | [blackbox_format.md](blackbox_format.md#encodeend) |
| `encodeFrameHeader` | free function | [blackbox_format.md](blackbox_format.md#encodeframeheader) |
| `encodeHeader` | free function | [blackbox_format.md](blackbox_format.md#encodeheader) |
| `encodeSummary` | free function | [blackbox_format.md](blackbox_format.md#encodesummary) |
| `encodeTick` | free function | [blackbox_format.md](blackbox_format.md#encodetick) |
| `encodeTriage` | free function | [blackbox_format.md](blackbox_format.md#encodetriage) |
| `EndInfo` | struct | [blackbox_format.md](blackbox_format.md#struct-endinfo) |
| `EndInfo::brownout` | field | [blackbox_format.md](blackbox_format.md#endinfo-brownout) |
| `EndInfo::bytesBefore` | field | [blackbox_format.md](blackbox_format.md#endinfo-bytesbefore) |
| `EndInfo::deviceFailed` | field | [blackbox_format.md](blackbox_format.md#endinfo-devicefailed) |
| `EndInfo::droppedFrames` | field | [blackbox_format.md](blackbox_format.md#endinfo-droppedframes) |
| `EndInfo::endTime` | field | [blackbox_format.md](blackbox_format.md#endinfo-endtime) |
| `EndInfo::messagesSeen` | field | [blackbox_format.md](blackbox_format.md#endinfo-messagesseen) |
| `EndInfo::tickFrames` | field | [blackbox_format.md](blackbox_format.md#endinfo-tickframes) |
| `ExitGroup` | class | [exit_group.md](exit_group.md#class-exitgroup) |
| `ExitGroup::check` | function | [exit_group.md](exit_group.md#exitgroup-check) |
| `ExitGroup::ExitGroup` | function | [exit_group.md](exit_group.md#exitgroup-exitgroup) |
| `ExitGroup::settled` | function | [exit_group.md](exit_group.md#exitgroup-settled) |
| `ExitGroup::start` | function | [exit_group.md](exit_group.md#exitgroup-start) |
| `ExitGroup::watchdog` | function | [exit_group.md](exit_group.md#exitgroup-watchdog) |
| `ExitReason` | enum class | [exit_group.md](exit_group.md#enum-class-exitreason) |
| `ExitReason::Cancelled` | enumerator | [exit_group.md](exit_group.md#exitreason-cancelled) |
| `ExitReason::Running` | enumerator | [exit_group.md](exit_group.md#exitreason-running) |
| `ExitReason::Settled` | enumerator | [exit_group.md](exit_group.md#exitreason-settled) |
| `ExitReason::TimedOut` | enumerator | [exit_group.md](exit_group.md#exitreason-timedout) |

## F

| Name | Kind | Page |
|---|---|---|
| `faultBit` | free function | [motion_scheduler.md](motion_scheduler.md#faultbit) |
| `FaultCode` | enum class | [fault.md](fault.md#enum-class-faultcode) |
| `FaultCode::Brownout` | enumerator | [fault.md](fault.md#faultcode-brownout) |
| `FaultCode::GpsGateReject` | enumerator | [fault.md](fault.md#faultcode-gpsgatereject) |
| `FaultCode::Implausible` | enumerator | [fault.md](fault.md#faultcode-implausible) |
| `FaultCode::ImuLost` | enumerator | [fault.md](fault.md#faultcode-imulost) |
| `FaultCode::LoopOverrun` | enumerator | [fault.md](fault.md#faultcode-loopoverrun) |
| `FaultCode::MechanismStalled` | enumerator | [fault.md](fault.md#faultcode-mechanismstalled) |
| `FaultCode::MotionTimeout` | enumerator | [fault.md](fault.md#faultcode-motiontimeout) |
| `FaultCode::MotorOverTemp` | enumerator | [fault.md](fault.md#faultcode-motorovertemp) |
| `FaultCode::NanPose` | enumerator | [fault.md](fault.md#faultcode-nanpose) |
| `FaultCode::None` | enumerator | [fault.md](fault.md#faultcode-none) |
| `FaultCode::OdoStuck` | enumerator | [fault.md](fault.md#faultcode-odostuck) |
| `FaultCode::Precondition` | enumerator | [fault.md](fault.md#faultcode-precondition) |
| `faultCodeName` | free function | [fault.md](fault.md#faultcodename) |
| `FaultLatch` | class | [fault.md](fault.md#class-faultlatch) |
| `FaultLatch::clear` | function | [fault.md](fault.md#faultlatch-clear) |
| `FaultLatch::faultCount` | function | [fault.md](fault.md#faultlatch-faultcount) |
| `FaultLatch::FaultLatch` | function | [fault.md](fault.md#faultlatch-faultlatch) |
| `FaultLatch::firstFault` | function | [fault.md](fault.md#faultlatch-firstfault) |
| `FaultLatch::firstFaultTime` | function | [fault.md](fault.md#faultlatch-firstfaulttime) |
| `FaultLatch::hasFault` | function | [fault.md](fault.md#faultlatch-hasfault) |
| `FaultLatch::lastFault` | function | [fault.md](fault.md#faultlatch-lastfault) |
| `FaultLatch::raise` | function | [fault.md](fault.md#faultlatch-raise) |
| `FaultLatch::raiseCount` | function | [fault.md](fault.md#faultlatch-raisecount) |
| `Feedforward` | class | [feedforward.md](feedforward.md#class-feedforward) |
| `Feedforward::calculate` | function | [feedforward.md](feedforward.md#feedforward-calculate) |
| `Feedforward::calculate (overload 2)` | function | [feedforward.md](feedforward.md#feedforward-calculate-2) |
| `Feedforward::Feedforward` | function | [feedforward.md](feedforward.md#feedforward-feedforward) |
| `FeedforwardGains` | struct | [feedforward.md](feedforward.md#struct-feedforwardgains) |
| `FeedforwardGains::kA` | field | [feedforward.md](feedforward.md#feedforwardgains-ka) |
| `FeedforwardGains::kS` | field | [feedforward.md](feedforward.md#feedforwardgains-ks) |
| `FeedforwardGains::kV` | field | [feedforward.md](feedforward.md#feedforwardgains-kv) |
| `FieldDelta` | struct | [arc_step.md](arc_step.md#struct-fielddelta) |
| `FieldDelta::dx` | field | [arc_step.md](arc_step.md#fielddelta-dx) |
| `FieldDelta::dy` | field | [arc_step.md](arc_step.md#fielddelta-dy) |
| `fieldToRobot` | free function | [frame.md](frame.md#fieldtorobot) |
| `Frame` | enum class | [frame.md](frame.md#enum-class-frame) |
| `Frame::Body` | enumerator | [frame.md](frame.md#frame-body) |
| `Frame::Field` | enumerator | [frame.md](frame.md#frame-field) |
| `FrameType` | enum class | [blackbox_format.md](blackbox_format.md#enum-class-frametype) |
| `FrameType::End` | enumerator | [blackbox_format.md](blackbox_format.md#frametype-end) |
| `FrameType::Summary` | enumerator | [blackbox_format.md](blackbox_format.md#frametype-summary) |
| `FrameType::Tick` | enumerator | [blackbox_format.md](blackbox_format.md#frametype-tick) |
| `FrameType::Triage` | enumerator | [blackbox_format.md](blackbox_format.md#frametype-triage) |
| `FusionResult` | struct | [correction.md](correction.md#struct-fusionresult) |
| `FusionResult::applied` | field | [correction.md](correction.md#fusionresult-applied) |
| `FusionResult::appliedConfidence` | field | [correction.md](correction.md#fusionresult-appliedconfidence) |
| `FusionResult::audit` | field | [correction.md](correction.md#fusionresult-audit) |
| `FusionResult::clamped` | field | [correction.md](correction.md#fusionresult-clamped) |
| `FusionResult::gated` | field | [correction.md](correction.md#fusionresult-gated) |
| `FusionResult::headingApplied` | field | [correction.md](correction.md#fusionresult-headingapplied) |
| `FusionResult::headingClamped` | field | [correction.md](correction.md#fusionresult-headingclamped) |
| `FusionResult::headingGated` | field | [correction.md](correction.md#fusionresult-headinggated) |
| `FusionResult::headingNudge` | field | [correction.md](correction.md#fusionresult-headingnudge) |
| `FusionResult::x` | field | [correction.md](correction.md#fusionresult-x) |
| `FusionResult::y` | field | [correction.md](correction.md#fusionresult-y) |

## G

| Name | Kind | Page |
|---|---|---|
| `GateAudit` | struct | [correction.md](correction.md#struct-gateaudit) |
| `GateAudit::covarianceTrace` | field | [correction.md](correction.md#gateaudit-covariancetrace) |
| `GateAudit::mahalanobis` | field | [correction.md](correction.md#gateaudit-mahalanobis) |
| `GateAudit::reason` | field | [correction.md](correction.md#gateaudit-reason) |
| `GateAudit::residualHeading` | field | [correction.md](correction.md#gateaudit-residualheading) |
| `GateAudit::residualX` | field | [correction.md](correction.md#gateaudit-residualx) |
| `GateAudit::residualY` | field | [correction.md](correction.md#gateaudit-residualy) |
| `GateReason` | enum class | [debug_record.md](debug_record.md#enum-class-gatereason) |
| `GateReason::Accepted` | enumerator | [debug_record.md](debug_record.md#gatereason-accepted) |
| `GateReason::CovarianceReinit` | enumerator | [debug_record.md](debug_record.md#gatereason-covariancereinit) |
| `GateReason::None` | enumerator | [debug_record.md](debug_record.md#gatereason-none) |
| `GateReason::RejectedHighYawRate` | enumerator | [debug_record.md](debug_record.md#gatereason-rejectedhighyawrate) |
| `GateReason::RejectedInnovation` | enumerator | [debug_record.md](debug_record.md#gatereason-rejectedinnovation) |
| `GateReason::RejectedMahalanobis` | enumerator | [debug_record.md](debug_record.md#gatereason-rejectedmahalanobis) |
| `GateReason::RejectedNoFix` | enumerator | [debug_record.md](debug_record.md#gatereason-rejectednofix) |
| `GateReason::RejectedNormalizedInnovation` | enumerator | [debug_record.md](debug_record.md#gatereason-rejectednormalizedinnovation) |
| `GateReason::RejectedNoTagMapEntry` | enumerator | [debug_record.md](debug_record.md#gatereason-rejectednotagmapentry) |
| `GateReason::RejectedObservationAge` | enumerator | [debug_record.md](debug_record.md#gatereason-rejectedobservationage) |
| `GateReason::RejectedSensorQuality` | enumerator | [debug_record.md](debug_record.md#gatereason-rejectedsensorquality) |
| `GateReason::RejectedStaleFix` | enumerator | [debug_record.md](debug_record.md#gatereason-rejectedstalefix) |
| `GateReason::RejectedTagRange` | enumerator | [debug_record.md](debug_record.md#gatereason-rejectedtagrange) |
| `GpsCorrector` | class | [gps_corrector.md](gps_corrector.md#class-gpscorrector) |
| `GpsCorrector::acceptedFixes` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-acceptedfixes) |
| `GpsCorrector::GpsCorrector` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-gpscorrector) |
| `GpsCorrector::innovationRejects` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-innovationrejects) |
| `GpsCorrector::kHistory` | field | [gps_corrector.md](gps_corrector.md#gpscorrector-khistory) |
| `GpsCorrector::lastVerdict` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-lastverdict) |
| `GpsCorrector::name` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-name) |
| `GpsCorrector::noFixTicks` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-nofixticks) |
| `GpsCorrector::propose` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-propose) |
| `GpsCorrector::qualityRejects` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-qualityrejects) |
| `GpsCorrector::staleTicks` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-staleticks) |
| `GpsCorrector::travelSinceFix` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-travelsincefix) |
| `GpsCorrector::yawRateRejects` | function | [gps_corrector.md](gps_corrector.md#gpscorrector-yawraterejects) |
| `GpsCorrectorConfig` | struct | [gps_corrector.md](gps_corrector.md#struct-gpscorrectorconfig) |
| `GpsCorrectorConfig::driftStdDevPerInch` | field | [gps_corrector.md](gps_corrector.md#gpscorrectorconfig-driftstddevperinch) |
| `GpsCorrectorConfig::gateSigma` | field | [gps_corrector.md](gps_corrector.md#gpscorrectorconfig-gatesigma) |
| `GpsCorrectorConfig::latency` | field | [gps_corrector.md](gps_corrector.md#gpscorrectorconfig-latency) |
| `GpsCorrectorConfig::maxReportedRms` | field | [gps_corrector.md](gps_corrector.md#gpscorrectorconfig-maxreportedrms) |
| `GpsCorrectorConfig::maxYawRate` | field | [gps_corrector.md](gps_corrector.md#gpscorrectorconfig-maxyawrate) |
| `GpsCorrectorConfig::minPositionStdDev` | field | [gps_corrector.md](gps_corrector.md#gpscorrectorconfig-minpositionstddev) |
| `GpsCorrectorConfig::postFixStdDev` | field | [gps_corrector.md](gps_corrector.md#gpscorrectorconfig-postfixstddev) |
| `GpsCorrectorConfig::rmsTrustFactor` | field | [gps_corrector.md](gps_corrector.md#gpscorrectorconfig-rmstrustfactor) |
| `gpsHeadingToCanonical` | free function | [gps_conversion.md](gps_conversion.md#gpsheadingtocanonical) |
| `gpsRemoveLeverArm` | free function | [gps_conversion.md](gps_conversion.md#gpsremoveleverarm) |
| `gpsRmsErrorToCanonical` | free function | [gps_conversion.md](gps_conversion.md#gpsrmserrortocanonical) |
| `gpsSensorPose` | free function | [gps_conversion.md](gps_conversion.md#gpssensorpose) |
| `gpsToRobotPose` | free function | [gps_conversion.md](gps_conversion.md#gpstorobotpose) |
| `GuardedWaitResult` | enum class | [run_guard.md](run_guard.md#enum-class-guardedwaitresult) |
| `GuardedWaitResult::RunExpired` | enumerator | [run_guard.md](run_guard.md#guardedwaitresult-runexpired) |
| `GuardedWaitResult::Satisfied` | enumerator | [run_guard.md](run_guard.md#guardedwaitresult-satisfied) |
| `GuardedWaitResult::TimedOut` | enumerator | [run_guard.md](run_guard.md#guardedwaitresult-timedout) |

## H

| Name | Kind | Page |
|---|---|---|
| `hDrive` | free function | [h_drive.md](h_drive.md#hdrive) |
| `HDriveConfig` | struct | [h_drive.md](h_drive.md#struct-hdriveconfig) |
| `HDriveConfig::strafeSpeedRatio` | field | [h_drive.md](h_drive.md#hdriveconfig-strafespeedratio) |
| `HDriveConfig::strafeTractionDerate` | field | [h_drive.md](h_drive.md#hdriveconfig-strafetractionderate) |
| `HDriveConfig::strafeWheelOffset` | field | [h_drive.md](h_drive.md#hdriveconfig-strafewheeloffset) |
| `HDriveConfig::trackWidth` | field | [h_drive.md](h_drive.md#hdriveconfig-trackwidth) |
| `HealthMonitor` | class | [health_monitor.md](health_monitor.md#class-healthmonitor) |
| `HealthMonitor::brownedOut` | function | [health_monitor.md](health_monitor.md#healthmonitor-brownedout) |
| `HealthMonitor::HealthMonitor` | function | [health_monitor.md](health_monitor.md#healthmonitor-healthmonitor) |
| `HealthMonitor::imuLost` | function | [health_monitor.md](health_monitor.md#healthmonitor-imulost) |
| `HealthMonitor::Observations` | struct | [health_monitor.md](health_monitor.md#struct-healthmonitor-observations) |
| `HealthMonitor::Observations::batteryVolts` | field | [health_monitor.md](health_monitor.md#healthmonitor-observations-batteryvolts) |
| `HealthMonitor::Observations::fixGated` | field | [health_monitor.md](health_monitor.md#healthmonitor-observations-fixgated) |
| `HealthMonitor::Observations::imuReady` | field | [health_monitor.md](health_monitor.md#healthmonitor-observations-imuready) |
| `HealthMonitor::Observations::maxMotorTempC` | field | [health_monitor.md](health_monitor.md#healthmonitor-observations-maxmotortempc) |
| `HealthMonitor::Observations::odomImplausible` | field | [health_monitor.md](health_monitor.md#healthmonitor-observations-odomimplausible) |
| `HealthMonitor::Observations::odomStalled` | field | [health_monitor.md](health_monitor.md#healthmonitor-observations-odomstalled) |
| `HealthMonitor::reset` | function | [health_monitor.md](health_monitor.md#healthmonitor-reset) |
| `HealthMonitor::tick` | function | [health_monitor.md](health_monitor.md#healthmonitor-tick) |
| `HealthMonitorConfig` | struct | [health_monitor.md](health_monitor.md#struct-healthmonitorconfig) |
| `HealthMonitorConfig::brownoutRecoverVolts` | field | [health_monitor.md](health_monitor.md#healthmonitorconfig-brownoutrecovervolts) |
| `HealthMonitorConfig::brownoutVolts` | field | [health_monitor.md](health_monitor.md#healthmonitorconfig-brownoutvolts) |
| `HealthMonitorConfig::maxMotorTempC` | field | [health_monitor.md](health_monitor.md#healthmonitorconfig-maxmotortempc) |
| `HoldPose` | class | [hold_pose.md](hold_pose.md#class-holdpose) |
| `HoldPose::HoldPose` | function | [hold_pose.md](hold_pose.md#holdpose-holdpose) |
| `HoldPose::HoldPose (overload 2)` | function | [hold_pose.md](hold_pose.md#holdpose-holdpose-2) |
| `HoldPose::name` | function | [hold_pose.md](hold_pose.md#holdpose-name) |

## I

| Name | Kind | Page |
|---|---|---|
| `IBattery` | class | [battery.md](battery.md#class-ibattery) |
| `IBattery::capacity` | function | [battery.md](battery.md#ibattery-capacity) |
| `IBattery::current` | function | [battery.md](battery.md#ibattery-current) |
| `IBattery::IBattery` | function | [battery.md](battery.md#ibattery-ibattery) |
| `IBattery::IBattery (overload 2)` | function | [battery.md](battery.md#ibattery-ibattery-2) |
| `IBattery::IBattery (overload 3)` | function | [battery.md](battery.md#ibattery-ibattery-3) |
| `IBattery::operator=` | function | [battery.md](battery.md#ibattery-operator-eq) |
| `IBattery::operator= (overload 2)` | function | [battery.md](battery.md#ibattery-operator-eq-2) |
| `IBattery::voltage` | function | [battery.md](battery.md#ibattery-voltage) |
| `IBattery::~IBattery` | function | [battery.md](battery.md#ibattery-destructor-ibattery) |
| `IBlockSink` | class | [block_sink.md](block_sink.md#class-iblocksink) |
| `IBlockSink::flush` | function | [block_sink.md](block_sink.md#iblocksink-flush) |
| `IBlockSink::IBlockSink` | function | [block_sink.md](block_sink.md#iblocksink-iblocksink) |
| `IBlockSink::IBlockSink (overload 2)` | function | [block_sink.md](block_sink.md#iblocksink-iblocksink-2) |
| `IBlockSink::IBlockSink (overload 3)` | function | [block_sink.md](block_sink.md#iblocksink-iblocksink-3) |
| `IBlockSink::operator=` | function | [block_sink.md](block_sink.md#iblocksink-operator-eq) |
| `IBlockSink::operator= (overload 2)` | function | [block_sink.md](block_sink.md#iblocksink-operator-eq-2) |
| `IBlockSink::write` | function | [block_sink.md](block_sink.md#iblocksink-write) |
| `IBlockSink::~IBlockSink` | function | [block_sink.md](block_sink.md#iblocksink-destructor-iblocksink) |
| `ICancellable` | class | [mechanism.md](mechanism.md#class-icancellable) |
| `ICancellable::cancel` | function | [mechanism.md](mechanism.md#icancellable-cancel) |
| `ICancellable::ICancellable` | function | [mechanism.md](mechanism.md#icancellable-icancellable) |
| `ICancellable::ICancellable (overload 2)` | function | [mechanism.md](mechanism.md#icancellable-icancellable-2) |
| `ICancellable::ICancellable (overload 3)` | function | [mechanism.md](mechanism.md#icancellable-icancellable-3) |
| `ICancellable::operator=` | function | [mechanism.md](mechanism.md#icancellable-operator-eq) |
| `ICancellable::operator= (overload 2)` | function | [mechanism.md](mechanism.md#icancellable-operator-eq-2) |
| `ICancellable::~ICancellable` | function | [mechanism.md](mechanism.md#icancellable-destructor-icancellable) |
| `ICharSink` | class | [char_sink.md](char_sink.md#class-icharsink) |
| `ICharSink::ICharSink` | function | [char_sink.md](char_sink.md#icharsink-icharsink) |
| `ICharSink::ICharSink (overload 2)` | function | [char_sink.md](char_sink.md#icharsink-icharsink-2) |
| `ICharSink::ICharSink (overload 3)` | function | [char_sink.md](char_sink.md#icharsink-icharsink-3) |
| `ICharSink::operator=` | function | [char_sink.md](char_sink.md#icharsink-operator-eq) |
| `ICharSink::operator= (overload 2)` | function | [char_sink.md](char_sink.md#icharsink-operator-eq-2) |
| `ICharSink::write` | function | [char_sink.md](char_sink.md#icharsink-write) |
| `ICharSink::~ICharSink` | function | [char_sink.md](char_sink.md#icharsink-destructor-icharsink) |
| `IClock` | class | [clock.md](clock.md#class-iclock) |
| `IClock::IClock` | function | [clock.md](clock.md#iclock-iclock) |
| `IClock::IClock (overload 2)` | function | [clock.md](clock.md#iclock-iclock-2) |
| `IClock::IClock (overload 3)` | function | [clock.md](clock.md#iclock-iclock-3) |
| `IClock::now` | function | [clock.md](clock.md#iclock-now) |
| `IClock::operator=` | function | [clock.md](clock.md#iclock-operator-eq) |
| `IClock::operator= (overload 2)` | function | [clock.md](clock.md#iclock-operator-eq-2) |
| `IClock::~IClock` | function | [clock.md](clock.md#iclock-destructor-iclock) |
| `IController` | class | [controller.md](controller.md#class-icontroller) |
| `IController::axis` | function | [controller.md](controller.md#icontroller-axis) |
| `IController::IController` | function | [controller.md](controller.md#icontroller-icontroller) |
| `IController::IController (overload 2)` | function | [controller.md](controller.md#icontroller-icontroller-2) |
| `IController::IController (overload 3)` | function | [controller.md](controller.md#icontroller-icontroller-3) |
| `IController::isConnected` | function | [controller.md](controller.md#icontroller-isconnected) |
| `IController::operator=` | function | [controller.md](controller.md#icontroller-operator-eq) |
| `IController::operator= (overload 2)` | function | [controller.md](controller.md#icontroller-operator-eq-2) |
| `IController::pressed` | function | [controller.md](controller.md#icontroller-pressed) |
| `IController::~IController` | function | [controller.md](controller.md#icontroller-destructor-icontroller) |
| `ICorrector` | class | [i_corrector.md](i_corrector.md#class-icorrector) |
| `ICorrector::ICorrector` | function | [i_corrector.md](i_corrector.md#icorrector-icorrector) |
| `ICorrector::ICorrector (overload 2)` | function | [i_corrector.md](i_corrector.md#icorrector-icorrector-2) |
| `ICorrector::ICorrector (overload 3)` | function | [i_corrector.md](i_corrector.md#icorrector-icorrector-3) |
| `ICorrector::name` | function | [i_corrector.md](i_corrector.md#icorrector-name) |
| `ICorrector::operator=` | function | [i_corrector.md](i_corrector.md#icorrector-operator-eq) |
| `ICorrector::operator= (overload 2)` | function | [i_corrector.md](i_corrector.md#icorrector-operator-eq-2) |
| `ICorrector::propose` | function | [i_corrector.md](i_corrector.md#icorrector-propose) |
| `ICorrector::~ICorrector` | function | [i_corrector.md](i_corrector.md#icorrector-destructor-icorrector) |
| `IDigitalIn` | class | [digital_in.md](digital_in.md#class-idigitalin) |
| `IDigitalIn::IDigitalIn` | function | [digital_in.md](digital_in.md#idigitalin-idigitalin) |
| `IDigitalIn::IDigitalIn (overload 2)` | function | [digital_in.md](digital_in.md#idigitalin-idigitalin-2) |
| `IDigitalIn::IDigitalIn (overload 3)` | function | [digital_in.md](digital_in.md#idigitalin-idigitalin-3) |
| `IDigitalIn::operator=` | function | [digital_in.md](digital_in.md#idigitalin-operator-eq) |
| `IDigitalIn::operator= (overload 2)` | function | [digital_in.md](digital_in.md#idigitalin-operator-eq-2) |
| `IDigitalIn::state` | function | [digital_in.md](digital_in.md#idigitalin-state) |
| `IDigitalIn::~IDigitalIn` | function | [digital_in.md](digital_in.md#idigitalin-destructor-idigitalin) |
| `IDigitalOut` | class | [digital_out.md](digital_out.md#class-idigitalout) |
| `IDigitalOut::commanded` | function | [digital_out.md](digital_out.md#idigitalout-commanded) |
| `IDigitalOut::IDigitalOut` | function | [digital_out.md](digital_out.md#idigitalout-idigitalout) |
| `IDigitalOut::IDigitalOut (overload 2)` | function | [digital_out.md](digital_out.md#idigitalout-idigitalout-2) |
| `IDigitalOut::IDigitalOut (overload 3)` | function | [digital_out.md](digital_out.md#idigitalout-idigitalout-3) |
| `IDigitalOut::operator=` | function | [digital_out.md](digital_out.md#idigitalout-operator-eq) |
| `IDigitalOut::operator= (overload 2)` | function | [digital_out.md](digital_out.md#idigitalout-operator-eq-2) |
| `IDigitalOut::set` | function | [digital_out.md](digital_out.md#idigitalout-set) |
| `IDigitalOut::~IDigitalOut` | function | [digital_out.md](digital_out.md#idigitalout-destructor-idigitalout) |
| `IDistance` | class | [distance.md](distance.md#class-idistance) |
| `IDistance::confidence` | function | [distance.md](distance.md#idistance-confidence) |
| `IDistance::distance` | function | [distance.md](distance.md#idistance-distance) |
| `IDistance::IDistance` | function | [distance.md](distance.md#idistance-idistance) |
| `IDistance::IDistance (overload 2)` | function | [distance.md](distance.md#idistance-idistance-2) |
| `IDistance::IDistance (overload 3)` | function | [distance.md](distance.md#idistance-idistance-3) |
| `IDistance::operator=` | function | [distance.md](distance.md#idistance-operator-eq) |
| `IDistance::operator= (overload 2)` | function | [distance.md](distance.md#idistance-operator-eq-2) |
| `IDistance::~IDistance` | function | [distance.md](distance.md#idistance-destructor-idistance) |
| `IFusionPolicy` | class | [i_fusion_policy.md](i_fusion_policy.md#class-ifusionpolicy) |
| `IFusionPolicy::fuse` | function | [i_fusion_policy.md](i_fusion_policy.md#ifusionpolicy-fuse) |
| `IFusionPolicy::IFusionPolicy` | function | [i_fusion_policy.md](i_fusion_policy.md#ifusionpolicy-ifusionpolicy) |
| `IFusionPolicy::IFusionPolicy (overload 2)` | function | [i_fusion_policy.md](i_fusion_policy.md#ifusionpolicy-ifusionpolicy-2) |
| `IFusionPolicy::IFusionPolicy (overload 3)` | function | [i_fusion_policy.md](i_fusion_policy.md#ifusionpolicy-ifusionpolicy-3) |
| `IFusionPolicy::operator=` | function | [i_fusion_policy.md](i_fusion_policy.md#ifusionpolicy-operator-eq) |
| `IFusionPolicy::operator= (overload 2)` | function | [i_fusion_policy.md](i_fusion_policy.md#ifusionpolicy-operator-eq-2) |
| `IFusionPolicy::~IFusionPolicy` | function | [i_fusion_policy.md](i_fusion_policy.md#ifusionpolicy-destructor-ifusionpolicy) |
| `IGps` | class | [gps.md](gps.md#class-igps) |
| `IGps::hasFix` | function | [gps.md](gps.md#igps-hasfix) |
| `IGps::IGps` | function | [gps.md](gps.md#igps-igps) |
| `IGps::IGps (overload 2)` | function | [gps.md](gps.md#igps-igps-2) |
| `IGps::IGps (overload 3)` | function | [gps.md](gps.md#igps-igps-3) |
| `IGps::operator=` | function | [gps.md](gps.md#igps-operator-eq) |
| `IGps::operator= (overload 2)` | function | [gps.md](gps.md#igps-operator-eq-2) |
| `IGps::pose` | function | [gps.md](gps.md#igps-pose) |
| `IGps::rmsError` | function | [gps.md](gps.md#igps-rmserror) |
| `IGps::~IGps` | function | [gps.md](gps.md#igps-destructor-igps) |
| `IImu` | class | [imu.md](imu.md#class-iimu) |
| `IImu::heading` | function | [imu.md](imu.md#iimu-heading) |
| `IImu::IImu` | function | [imu.md](imu.md#iimu-iimu) |
| `IImu::IImu (overload 2)` | function | [imu.md](imu.md#iimu-iimu-2) |
| `IImu::IImu (overload 3)` | function | [imu.md](imu.md#iimu-iimu-3) |
| `IImu::isReady` | function | [imu.md](imu.md#iimu-isready) |
| `IImu::operator=` | function | [imu.md](imu.md#iimu-operator-eq) |
| `IImu::operator= (overload 2)` | function | [imu.md](imu.md#iimu-operator-eq-2) |
| `IImu::pitch` | function | [imu.md](imu.md#iimu-pitch) |
| `IImu::roll` | function | [imu.md](imu.md#iimu-roll) |
| `IImu::yawRate` | function | [imu.md](imu.md#iimu-yawrate) |
| `IImu::~IImu` | function | [imu.md](imu.md#iimu-destructor-iimu) |
| `IKinematics` | class | [kinematics.md](kinematics.md#class-ikinematics) |
| `IKinematics::desaturate` | function | [kinematics.md](kinematics.md#ikinematics-desaturate) |
| `IKinematics::forward` | function | [kinematics.md](kinematics.md#ikinematics-forward) |
| `IKinematics::IKinematics` | function | [kinematics.md](kinematics.md#ikinematics-ikinematics) |
| `IKinematics::IKinematics (overload 2)` | function | [kinematics.md](kinematics.md#ikinematics-ikinematics-2) |
| `IKinematics::IKinematics (overload 3)` | function | [kinematics.md](kinematics.md#ikinematics-ikinematics-3) |
| `IKinematics::operator=` | function | [kinematics.md](kinematics.md#ikinematics-operator-eq) |
| `IKinematics::operator= (overload 2)` | function | [kinematics.md](kinematics.md#ikinematics-operator-eq-2) |
| `IKinematics::strafeAuthority` | function | [kinematics.md](kinematics.md#ikinematics-strafeauthority) |
| `IKinematics::toWheels` | function | [kinematics.md](kinematics.md#ikinematics-towheels) |
| `IKinematics::wheelCount` | function | [kinematics.md](kinematics.md#ikinematics-wheelcount) |
| `IKinematics::~IKinematics` | function | [kinematics.md](kinematics.md#ikinematics-destructor-ikinematics) |
| `ILineDisplay` | class | [line_display.md](line_display.md#class-ilinedisplay) |
| `ILineDisplay::ILineDisplay` | function | [line_display.md](line_display.md#ilinedisplay-ilinedisplay) |
| `ILineDisplay::ILineDisplay (overload 2)` | function | [line_display.md](line_display.md#ilinedisplay-ilinedisplay-2) |
| `ILineDisplay::ILineDisplay (overload 3)` | function | [line_display.md](line_display.md#ilinedisplay-ilinedisplay-3) |
| `ILineDisplay::kCols` | field | [line_display.md](line_display.md#ilinedisplay-kcols) |
| `ILineDisplay::kRows` | field | [line_display.md](line_display.md#ilinedisplay-krows) |
| `ILineDisplay::operator=` | function | [line_display.md](line_display.md#ilinedisplay-operator-eq) |
| `ILineDisplay::operator= (overload 2)` | function | [line_display.md](line_display.md#ilinedisplay-operator-eq-2) |
| `ILineDisplay::setLine` | function | [line_display.md](line_display.md#ilinedisplay-setline) |
| `ILineDisplay::~ILineDisplay` | function | [line_display.md](line_display.md#ilinedisplay-destructor-ilinedisplay) |
| `IMechanism` | class | [mechanism.md](mechanism.md#class-imechanism) |
| `IMechanism::applySafeState` | function | [mechanism.md](mechanism.md#imechanism-applysafestate) |
| `IMechanism::claimant` | function | [mechanism.md](mechanism.md#imechanism-claimant) |
| `IMechanism::claimed` | function | [mechanism.md](mechanism.md#imechanism-claimed) |
| `IMechanism::IMechanism` | function | [mechanism.md](mechanism.md#imechanism-imechanism) |
| `IMechanism::IMechanism (overload 2)` | function | [mechanism.md](mechanism.md#imechanism-imechanism-2) |
| `IMechanism::IMechanism (overload 3)` | function | [mechanism.md](mechanism.md#imechanism-imechanism-3) |
| `IMechanism::name` | function | [mechanism.md](mechanism.md#imechanism-name) |
| `IMechanism::operator=` | function | [mechanism.md](mechanism.md#imechanism-operator-eq) |
| `IMechanism::operator= (overload 2)` | function | [mechanism.md](mechanism.md#imechanism-operator-eq-2) |
| `IMechanism::releaseClaim` | function | [mechanism.md](mechanism.md#imechanism-releaseclaim) |
| `IMechanism::tryClaim` | function | [mechanism.md](mechanism.md#imechanism-tryclaim) |
| `IMechanism::tryClaim (overload 2)` | function | [mechanism.md](mechanism.md#imechanism-tryclaim-2) |
| `IMechanism::~IMechanism` | function | [mechanism.md](mechanism.md#imechanism-destructor-imechanism) |
| `IMechanismOp` | class | [mechanism_op.md](mechanism_op.md#class-imechanismop) |
| `IMechanismOp::cancel` | function | [mechanism_op.md](mechanism_op.md#imechanismop-cancel) |
| `IMechanismOp::finished` | function | [mechanism_op.md](mechanism_op.md#imechanismop-finished) |
| `IMechanismOp::IMechanismOp` | function | [mechanism_op.md](mechanism_op.md#imechanismop-imechanismop) |
| `IMechanismOp::IMechanismOp (overload 2)` | function | [mechanism_op.md](mechanism_op.md#imechanismop-imechanismop-2) |
| `IMechanismOp::IMechanismOp (overload 3)` | function | [mechanism_op.md](mechanism_op.md#imechanismop-imechanismop-3) |
| `IMechanismOp::name` | function | [mechanism_op.md](mechanism_op.md#imechanismop-name) |
| `IMechanismOp::operator=` | function | [mechanism_op.md](mechanism_op.md#imechanismop-operator-eq) |
| `IMechanismOp::operator= (overload 2)` | function | [mechanism_op.md](mechanism_op.md#imechanismop-operator-eq-2) |
| `IMechanismOp::outcome` | function | [mechanism_op.md](mechanism_op.md#imechanismop-outcome) |
| `IMechanismOp::start` | function | [mechanism_op.md](mechanism_op.md#imechanismop-start) |
| `IMechanismOp::started` | function | [mechanism_op.md](mechanism_op.md#imechanismop-started) |
| `IMechanismOp::tick` | function | [mechanism_op.md](mechanism_op.md#imechanismop-tick) |
| `IMechanismOp::~IMechanismOp` | function | [mechanism_op.md](mechanism_op.md#imechanismop-destructor-imechanismop) |
| `IMotion` | class | [motion.md](motion.md#class-imotion) |
| `IMotion::cancel` | function | [motion.md](motion.md#imotion-cancel) |
| `IMotion::exitReason` | function | [motion.md](motion.md#imotion-exitreason) |
| `IMotion::IMotion` | function | [motion.md](motion.md#imotion-imotion) |
| `IMotion::IMotion (overload 2)` | function | [motion.md](motion.md#imotion-imotion-2) |
| `IMotion::IMotion (overload 3)` | function | [motion.md](motion.md#imotion-imotion-3) |
| `IMotion::name` | function | [motion.md](motion.md#imotion-name) |
| `IMotion::operator=` | function | [motion.md](motion.md#imotion-operator-eq) |
| `IMotion::operator= (overload 2)` | function | [motion.md](motion.md#imotion-operator-eq-2) |
| `IMotion::start` | function | [motion.md](motion.md#imotion-start) |
| `IMotion::state` | function | [motion.md](motion.md#imotion-state) |
| `IMotion::tick` | function | [motion.md](motion.md#imotion-tick) |
| `IMotion::~IMotion` | function | [motion.md](motion.md#imotion-destructor-imotion) |
| `IMotionObserver` | class | [motion_scheduler.md](motion_scheduler.md#class-imotionobserver) |
| `IMotionObserver::IMotionObserver` | function | [motion_scheduler.md](motion_scheduler.md#imotionobserver-imotionobserver) |
| `IMotionObserver::IMotionObserver (overload 2)` | function | [motion_scheduler.md](motion_scheduler.md#imotionobserver-imotionobserver-2) |
| `IMotionObserver::IMotionObserver (overload 3)` | function | [motion_scheduler.md](motion_scheduler.md#imotionobserver-imotionobserver-3) |
| `IMotionObserver::onMotionComplete` | function | [motion_scheduler.md](motion_scheduler.md#imotionobserver-onmotioncomplete) |
| `IMotionObserver::operator=` | function | [motion_scheduler.md](motion_scheduler.md#imotionobserver-operator-eq) |
| `IMotionObserver::operator= (overload 2)` | function | [motion_scheduler.md](motion_scheduler.md#imotionobserver-operator-eq-2) |
| `IMotionObserver::~IMotionObserver` | function | [motion_scheduler.md](motion_scheduler.md#imotionobserver-destructor-imotionobserver) |
| `IMotor` | class | [motor.md](motor.md#class-imotor) |
| `IMotor::brakeMode` | function | [motor.md](motor.md#imotor-brakemode) |
| `IMotor::commandedVoltage` | function | [motor.md](motor.md#imotor-commandedvoltage) |
| `IMotor::current` | function | [motor.md](motor.md#imotor-current) |
| `IMotor::IMotor` | function | [motor.md](motor.md#imotor-imotor) |
| `IMotor::IMotor (overload 2)` | function | [motor.md](motor.md#imotor-imotor-2) |
| `IMotor::IMotor (overload 3)` | function | [motor.md](motor.md#imotor-imotor-3) |
| `IMotor::operator=` | function | [motor.md](motor.md#imotor-operator-eq) |
| `IMotor::operator= (overload 2)` | function | [motor.md](motor.md#imotor-operator-eq-2) |
| `IMotor::position` | function | [motor.md](motor.md#imotor-position) |
| `IMotor::setBrakeMode` | function | [motor.md](motor.md#imotor-setbrakemode) |
| `IMotor::setVoltage` | function | [motor.md](motor.md#imotor-setvoltage) |
| `IMotor::temperature` | function | [motor.md](motor.md#imotor-temperature) |
| `IMotor::velocity` | function | [motor.md](motor.md#imotor-velocity) |
| `IMotor::~IMotor` | function | [motor.md](motor.md#imotor-destructor-imotor) |
| `imuHeadingToCanonical` | free function | [imu_conversion.md](imu_conversion.md#imuheadingtocanonical) |
| `imuYawRateToCanonical` | free function | [imu_conversion.md](imu_conversion.md#imuyawratetocanonical) |
| `IOptical` | class | [optical.md](optical.md#class-ioptical) |
| `IOptical::brightness` | function | [optical.md](optical.md#ioptical-brightness) |
| `IOptical::hue` | function | [optical.md](optical.md#ioptical-hue) |
| `IOptical::IOptical` | function | [optical.md](optical.md#ioptical-ioptical) |
| `IOptical::IOptical (overload 2)` | function | [optical.md](optical.md#ioptical-ioptical-2) |
| `IOptical::IOptical (overload 3)` | function | [optical.md](optical.md#ioptical-ioptical-3) |
| `IOptical::operator=` | function | [optical.md](optical.md#ioptical-operator-eq) |
| `IOptical::operator= (overload 2)` | function | [optical.md](optical.md#ioptical-operator-eq-2) |
| `IOptical::proximity` | function | [optical.md](optical.md#ioptical-proximity) |
| `IOptical::saturation` | function | [optical.md](optical.md#ioptical-saturation) |
| `IOptical::~IOptical` | function | [optical.md](optical.md#ioptical-destructor-ioptical) |
| `IPoseSource` | class | [i_pose_source.md](i_pose_source.md#class-iposesource) |
| `IPoseSource::IPoseSource` | function | [i_pose_source.md](i_pose_source.md#iposesource-iposesource) |
| `IPoseSource::IPoseSource (overload 2)` | function | [i_pose_source.md](i_pose_source.md#iposesource-iposesource-2) |
| `IPoseSource::IPoseSource (overload 3)` | function | [i_pose_source.md](i_pose_source.md#iposesource-iposesource-3) |
| `IPoseSource::isDeadReckoning` | function | [i_pose_source.md](i_pose_source.md#iposesource-isdeadreckoning) |
| `IPoseSource::operator=` | function | [i_pose_source.md](i_pose_source.md#iposesource-operator-eq) |
| `IPoseSource::operator= (overload 2)` | function | [i_pose_source.md](i_pose_source.md#iposesource-operator-eq-2) |
| `IPoseSource::pose` | function | [i_pose_source.md](i_pose_source.md#iposesource-pose) |
| `IPoseSource::quality` | function | [i_pose_source.md](i_pose_source.md#iposesource-quality) |
| `IPoseSource::twist` | function | [i_pose_source.md](i_pose_source.md#iposesource-twist) |
| `IPoseSource::~IPoseSource` | function | [i_pose_source.md](i_pose_source.md#iposesource-destructor-iposesource) |
| `IRotation` | class | [rotation.md](rotation.md#class-irotation) |
| `IRotation::IRotation` | function | [rotation.md](rotation.md#irotation-irotation) |
| `IRotation::IRotation (overload 2)` | function | [rotation.md](rotation.md#irotation-irotation-2) |
| `IRotation::IRotation (overload 3)` | function | [rotation.md](rotation.md#irotation-irotation-3) |
| `IRotation::operator=` | function | [rotation.md](rotation.md#irotation-operator-eq) |
| `IRotation::operator= (overload 2)` | function | [rotation.md](rotation.md#irotation-operator-eq-2) |
| `IRotation::position` | function | [rotation.md](rotation.md#irotation-position) |
| `IRotation::velocity` | function | [rotation.md](rotation.md#irotation-velocity) |
| `IRotation::~IRotation` | function | [rotation.md](rotation.md#irotation-destructor-irotation) |
| `isFinitePose` | free function | [finite_guard.md](finite_guard.md#isfinitepose) |
| `ITagSource` | class | [vision.md](vision.md#class-itagsource) |
| `ITagSource::ITagSource` | function | [vision.md](vision.md#itagsource-itagsource) |
| `ITagSource::ITagSource (overload 2)` | function | [vision.md](vision.md#itagsource-itagsource-2) |
| `ITagSource::ITagSource (overload 3)` | function | [vision.md](vision.md#itagsource-itagsource-3) |
| `ITagSource::operator=` | function | [vision.md](vision.md#itagsource-operator-eq) |
| `ITagSource::operator= (overload 2)` | function | [vision.md](vision.md#itagsource-operator-eq-2) |
| `ITagSource::tags` | function | [vision.md](vision.md#itagsource-tags) |
| `ITagSource::~ITagSource` | function | [vision.md](vision.md#itagsource-destructor-itagsource) |
| `ITelemetrySink` | class | [telemetry_sink.md](telemetry_sink.md#class-itelemetrysink) |
| `ITelemetrySink::emit` | function | [telemetry_sink.md](telemetry_sink.md#itelemetrysink-emit) |
| `ITelemetrySink::ITelemetrySink` | function | [telemetry_sink.md](telemetry_sink.md#itelemetrysink-itelemetrysink) |
| `ITelemetrySink::ITelemetrySink (overload 2)` | function | [telemetry_sink.md](telemetry_sink.md#itelemetrysink-itelemetrysink-2) |
| `ITelemetrySink::ITelemetrySink (overload 3)` | function | [telemetry_sink.md](telemetry_sink.md#itelemetrysink-itelemetrysink-3) |
| `ITelemetrySink::log` | function | [telemetry_sink.md](telemetry_sink.md#itelemetrysink-log) |
| `ITelemetrySink::operator=` | function | [telemetry_sink.md](telemetry_sink.md#itelemetrysink-operator-eq) |
| `ITelemetrySink::operator= (overload 2)` | function | [telemetry_sink.md](telemetry_sink.md#itelemetrysink-operator-eq-2) |
| `ITelemetrySink::summarize` | function | [telemetry_sink.md](telemetry_sink.md#itelemetrysink-summarize) |
| `ITelemetrySink::wantsRecord` | function | [telemetry_sink.md](telemetry_sink.md#itelemetrysink-wantsrecord) |
| `ITelemetrySink::~ITelemetrySink` | function | [telemetry_sink.md](telemetry_sink.md#itelemetrysink-destructor-itelemetrysink) |
| `ITickPacer` | class | [motion_scheduler.md](motion_scheduler.md#class-itickpacer) |
| `ITickPacer::ITickPacer` | function | [motion_scheduler.md](motion_scheduler.md#itickpacer-itickpacer) |
| `ITickPacer::ITickPacer (overload 2)` | function | [motion_scheduler.md](motion_scheduler.md#itickpacer-itickpacer-2) |
| `ITickPacer::ITickPacer (overload 3)` | function | [motion_scheduler.md](motion_scheduler.md#itickpacer-itickpacer-3) |
| `ITickPacer::operator=` | function | [motion_scheduler.md](motion_scheduler.md#itickpacer-operator-eq) |
| `ITickPacer::operator= (overload 2)` | function | [motion_scheduler.md](motion_scheduler.md#itickpacer-operator-eq-2) |
| `ITickPacer::pace` | function | [motion_scheduler.md](motion_scheduler.md#itickpacer-pace) |
| `ITickPacer::~ITickPacer` | function | [motion_scheduler.md](motion_scheduler.md#itickpacer-destructor-itickpacer) |
| `IVision` | class | [vision.md](vision.md#class-ivision) |
| `IVision::IVision` | function | [vision.md](vision.md#ivision-ivision) |
| `IVision::IVision (overload 2)` | function | [vision.md](vision.md#ivision-ivision-2) |
| `IVision::IVision (overload 3)` | function | [vision.md](vision.md#ivision-ivision-3) |
| `IVision::objects` | function | [vision.md](vision.md#ivision-objects) |
| `IVision::operator=` | function | [vision.md](vision.md#ivision-operator-eq) |
| `IVision::operator= (overload 2)` | function | [vision.md](vision.md#ivision-operator-eq-2) |
| `IVision::~IVision` | function | [vision.md](vision.md#ivision-destructor-ivision) |

## K

| Name | Kind | Page |
|---|---|---|
| `kApiMajor` | constant | [version.md](version.md#kapimajor) |
| `kApiMinor` | constant | [version.md](version.md#kapiminor) |
| `kApiVersionString` | constant | [version.md](version.md#kapiversionstring) |
| `kArcStraightEps` | constant | [arc_step.md](arc_step.md#karcstraighteps) |
| `kCommandAuditMargin` | constant | [plausibility_guard.md](plausibility_guard.md#kcommandauditmargin) |
| `kCompactThresholdBytes` | constant | [line_format.md](line_format.md#kcompactthresholdbytes) |
| `kDefaultFlightRingTicks` | constant | [sd_sink.md](sd_sink.md#kdefaultflightringticks) |
| `kDistanceConfidenceAvailableAboveMm` | constant | [distance_conversion.md](distance_conversion.md#kdistanceconfidenceavailableabovemm) |
| `kDistanceConfidenceFullScale` | constant | [distance_conversion.md](distance_conversion.md#kdistanceconfidencefullscale) |
| `kDistanceNoObjectMm` | constant | [distance_conversion.md](distance_conversion.md#kdistancenoobjectmm) |
| `kDockedHeadingTypicalDeg` | constant | [accuracy.md](accuracy.md#kdockedheadingtypicaldeg) |
| `kDockedPositionError` | constant | [accuracy.md](accuracy.md#kdockedpositionerror) |
| `kEndPayloadBytes` | constant | [blackbox_format.md](blackbox_format.md#kendpayloadbytes) |
| `kFormatVersion` | constant | [blackbox_format.md](blackbox_format.md#kformatversion) |
| `kFrameHeaderBytes` | constant | [blackbox_format.md](blackbox_format.md#kframeheaderbytes) |
| `kGpsDefaultNorthHeadingDeg` | constant | [gps_conversion.md](gps_conversion.md#kgpsdefaultnorthheadingdeg) |
| `kHeaderBytes` | constant | [blackbox_format.md](blackbox_format.md#kheaderbytes) |
| `kHeadingErrorMaxDeg` | constant | [accuracy.md](accuracy.md#kheadingerrormaxdeg) |
| `kMagic` | constant | [blackbox_format.md](blackbox_format.md#kmagic) |
| `kMaxFieldBytes` | constant | [session_info.md](session_info.md#kmaxfieldbytes) |
| `kMaxHashBytes` | constant | [session_info.md](session_info.md#kmaxhashbytes) |
| `kMaxMotorVoltage` | constant | [motor.md](motor.md#kmaxmotorvoltage) |
| `kMaxPortMapBytes` | constant | [session_info.md](session_info.md#kmaxportmapbytes) |
| `kMetersToInches` | constant | [gps_conversion.md](gps_conversion.md#kmeterstoinches) |
| `kPositionErrorEndOfRun` | constant | [accuracy.md](accuracy.md#kpositionerrorendofrun) |
| `kRecommendedBufferBytes` | constant | [sd_sink.md](sd_sink.md#krecommendedbufferbytes) |
| `kRepeatability` | constant | [accuracy.md](accuracy.md#krepeatability) |
| `kStrafeFallbackNoiseFraction` | constant | [command_pipeline.md](command_pipeline.md#kstrafefallbacknoisefraction) |
| `kSummaryPayloadBytes` | constant | [blackbox_format.md](blackbox_format.md#ksummarypayloadbytes) |
| `kTickPayloadBytes` | constant | [blackbox_format.md](blackbox_format.md#ktickpayloadbytes) |
| `kTickPhaseSlots` | constant | [debug_record.md](debug_record.md#ktickphaseslots) |
| `kTriagePayloadBytes` | constant | [blackbox_format.md](blackbox_format.md#ktriagepayloadbytes) |

## L

| Name | Kind | Page |
|---|---|---|
| `Length` | type alias | [quantity.md](quantity.md#length) |
| `LevelFilterSink` | class | [level_filter_sink.md](level_filter_sink.md#class-levelfiltersink) |
| `LevelFilterSink::clearLevels` | function | [level_filter_sink.md](level_filter_sink.md#levelfiltersink-clearlevels) |
| `LevelFilterSink::emit` | function | [level_filter_sink.md](level_filter_sink.md#levelfiltersink-emit) |
| `LevelFilterSink::LevelFilterSink` | function | [level_filter_sink.md](level_filter_sink.md#levelfiltersink-levelfiltersink) |
| `LevelFilterSink::log` | function | [level_filter_sink.md](level_filter_sink.md#levelfiltersink-log) |
| `LevelFilterSink::setGlobalLevel` | function | [level_filter_sink.md](level_filter_sink.md#levelfiltersink-setgloballevel) |
| `LevelFilterSink::setLevel` | function | [level_filter_sink.md](level_filter_sink.md#levelfiltersink-setlevel) |
| `LevelFilterSink::summarize` | function | [level_filter_sink.md](level_filter_sink.md#levelfiltersink-summarize) |
| `LevelFilterSink::wantsRecord` | function | [level_filter_sink.md](level_filter_sink.md#levelfiltersink-wantsrecord) |
| `Line` | struct | [line_format.md](line_format.md#struct-line) |
| `Line::appendLiteral` | function | [line_format.md](line_format.md#line-appendliteral) |
| `Line::appendRaw` | function | [line_format.md](line_format.md#line-appendraw) |
| `Line::appendSanitized` | function | [line_format.md](line_format.md#line-appendsanitized) |
| `Line::buf` | field | [line_format.md](line_format.md#line-buf) |
| `Line::kCapacity` | field | [line_format.md](line_format.md#line-kcapacity) |
| `Line::n` | field | [line_format.md](line_format.md#line-n) |
| `Line::view` | function | [line_format.md](line_format.md#line-view) |
| `Localizer` | class | [localizer.md](localizer.md#class-localizer) |
| `Localizer::distanceSinceCorrection` | function | [localizer.md](localizer.md#localizer-distancesincecorrection) |
| `Localizer::headingBias` | function | [localizer.md](localizer.md#localizer-headingbias) |
| `Localizer::isDeadReckoning` | function | [localizer.md](localizer.md#localizer-isdeadreckoning) |
| `Localizer::kMaxCorrectors` | field | [localizer.md](localizer.md#localizer-kmaxcorrectors) |
| `Localizer::lastCorrection` | function | [localizer.md](localizer.md#localizer-lastcorrection) |
| `Localizer::lastOdomDeltaImplausible` | function | [localizer.md](localizer.md#localizer-lastodomdeltaimplausible) |
| `Localizer::Localizer` | function | [localizer.md](localizer.md#localizer-localizer) |
| `Localizer::pose` | function | [localizer.md](localizer.md#localizer-pose) |
| `Localizer::Quality` | enum class | [localizer.md](localizer.md#enum-class-localizer-quality) |
| `Localizer::quality` | function | [localizer.md](localizer.md#localizer-quality) |
| `Localizer::Quality::Corrected` | enumerator | [localizer.md](localizer.md#localizer-quality-corrected) |
| `Localizer::Quality::DeadReckon` | enumerator | [localizer.md](localizer.md#localizer-quality-deadreckon) |
| `Localizer::Quality::Degraded` | enumerator | [localizer.md](localizer.md#localizer-quality-degraded) |
| `Localizer::Quality::Uninitialized` | enumerator | [localizer.md](localizer.md#localizer-quality-uninitialized) |
| `Localizer::qualityClass` | function | [localizer.md](localizer.md#localizer-qualityclass) |
| `Localizer::setPose` | function | [localizer.md](localizer.md#localizer-setpose) |
| `Localizer::twist` | function | [localizer.md](localizer.md#localizer-twist) |
| `Localizer::update` | function | [localizer.md](localizer.md#localizer-update) |
| `LocalizerConfig` | struct | [localizer.md](localizer.md#struct-localizerconfig) |
| `LocalizerConfig::bootSettleTime` | field | [localizer.md](localizer.md#localizerconfig-bootsettletime) |
| `LocalizerConfig::driftHorizon` | field | [localizer.md](localizer.md#localizerconfig-drifthorizon) |
| `LocalizerConfig::maxDt` | field | [localizer.md](localizer.md#localizerconfig-maxdt) |
| `LocalizerConfig::minDt` | field | [localizer.md](localizer.md#localizerconfig-mindt) |
| `LocalizerConfig::qFloor` | field | [localizer.md](localizer.md#localizerconfig-qfloor) |
| `LogLevel` | enum class | [telemetry_sink.md](telemetry_sink.md#enum-class-loglevel) |
| `LogLevel::Debug` | enumerator | [telemetry_sink.md](telemetry_sink.md#loglevel-debug) |
| `LogLevel::Error` | enumerator | [telemetry_sink.md](telemetry_sink.md#loglevel-error) |
| `LogLevel::Info` | enumerator | [telemetry_sink.md](telemetry_sink.md#loglevel-info) |
| `LogLevel::Trace` | enumerator | [telemetry_sink.md](telemetry_sink.md#loglevel-trace) |
| `LogLevel::Warn` | enumerator | [telemetry_sink.md](telemetry_sink.md#loglevel-warn) |
| `LoopMonitor` | class | [loop_monitor.md](loop_monitor.md#class-loopmonitor) |
| `LoopMonitor::LoopMonitor` | function | [loop_monitor.md](loop_monitor.md#loopmonitor-loopmonitor) |
| `LoopMonitor::overrunCount` | function | [loop_monitor.md](loop_monitor.md#loopmonitor-overruncount) |
| `LoopMonitor::reset` | function | [loop_monitor.md](loop_monitor.md#loopmonitor-reset) |
| `LoopMonitor::tick` | function | [loop_monitor.md](loop_monitor.md#loopmonitor-tick) |
| `LoopMonitor::worstDt` | function | [loop_monitor.md](loop_monitor.md#loopmonitor-worstdt) |
| `LoopMonitorConfig` | struct | [loop_monitor.md](loop_monitor.md#struct-loopmonitorconfig) |
| `LoopMonitorConfig::budget` | field | [loop_monitor.md](loop_monitor.md#loopmonitorconfig-budget) |

## M

| Name | Kind | Page |
|---|---|---|
| `MatrixKinematics` | class | [matrix_kinematics.md](matrix_kinematics.md#class-matrixkinematics) |
| `MatrixKinematics::desaturate` | function | [matrix_kinematics.md](matrix_kinematics.md#matrixkinematics-desaturate) |
| `MatrixKinematics::forward` | function | [matrix_kinematics.md](matrix_kinematics.md#matrixkinematics-forward) |
| `MatrixKinematics::MatrixKinematics` | function | [matrix_kinematics.md](matrix_kinematics.md#matrixkinematics-matrixkinematics) |
| `MatrixKinematics::strafeAuthority` | function | [matrix_kinematics.md](matrix_kinematics.md#matrixkinematics-strafeauthority) |
| `MatrixKinematics::toWheels` | function | [matrix_kinematics.md](matrix_kinematics.md#matrixkinematics-towheels) |
| `MatrixKinematics::Wheel` | struct | [matrix_kinematics.md](matrix_kinematics.md#struct-matrixkinematics-wheel) |
| `MatrixKinematics::Wheel::h` | field | [matrix_kinematics.md](matrix_kinematics.md#matrixkinematics-wheel-h) |
| `MatrixKinematics::Wheel::turnInches` | field | [matrix_kinematics.md](matrix_kinematics.md#matrixkinematics-wheel-turninches) |
| `MatrixKinematics::Wheel::v` | field | [matrix_kinematics.md](matrix_kinematics.md#matrixkinematics-wheel-v) |
| `MatrixKinematics::wheelCount` | function | [matrix_kinematics.md](matrix_kinematics.md#matrixkinematics-wheelcount) |
| `MechanismDeps` | struct | [mechanism_op.md](mechanism_op.md#struct-mechanismdeps) |
| `MechanismDeps::clock` | field | [mechanism_op.md](mechanism_op.md#mechanismdeps-clock) |
| `MechanismDeps::faults` | field | [mechanism_op.md](mechanism_op.md#mechanismdeps-faults) |
| `MechanismDeps::telemetry` | field | [mechanism_op.md](mechanism_op.md#mechanismdeps-telemetry) |
| `MechanismDeps::validate` | function | [mechanism_op.md](mechanism_op.md#mechanismdeps-validate) |
| `MechanismDeps::validatedClock` | function | [mechanism_op.md](mechanism_op.md#mechanismdeps-validatedclock) |
| `MechanismOutcome` | enum class | [mechanism_outcome.md](mechanism_outcome.md#enum-class-mechanismoutcome) |
| `MechanismOutcome::Cancelled` | enumerator | [mechanism_outcome.md](mechanism_outcome.md#mechanismoutcome-cancelled) |
| `MechanismOutcome::Running` | enumerator | [mechanism_outcome.md](mechanism_outcome.md#mechanismoutcome-running) |
| `MechanismOutcome::Stalled` | enumerator | [mechanism_outcome.md](mechanism_outcome.md#mechanismoutcome-stalled) |
| `MechanismOutcome::Succeeded` | enumerator | [mechanism_outcome.md](mechanism_outcome.md#mechanismoutcome-succeeded) |
| `MechanismOutcome::TimedOut` | enumerator | [mechanism_outcome.md](mechanism_outcome.md#mechanismoutcome-timedout) |
| `MechanismOutcome::Unconfirmed` | enumerator | [mechanism_outcome.md](mechanism_outcome.md#mechanismoutcome-unconfirmed) |
| `mechanismOutcomeName` | free function | [mechanism_outcome.md](mechanism_outcome.md#mechanismoutcomename) |
| `MotionConfig` | struct | [motion_config.md](motion_config.md#struct-motionconfig) |
| `MotionConfig::brakeSettle` | field | [motion_config.md](motion_config.md#motionconfig-brakesettle) |
| `MotionConfig::defaultTimeout` | field | [motion_config.md](motion_config.md#motionconfig-defaulttimeout) |
| `MotionConfig::heading` | field | [motion_config.md](motion_config.md#motionconfig-heading) |
| `MotionConfig::headingSettle` | field | [motion_config.md](motion_config.md#motionconfig-headingsettle) |
| `MotionConfig::maxAngularSpeed` | field | [motion_config.md](motion_config.md#motionconfig-maxangularspeed) |
| `MotionConfig::maxLinearSpeed` | field | [motion_config.md](motion_config.md#motionconfig-maxlinearspeed) |
| `MotionConfig::maxWheelSpeed` | field | [motion_config.md](motion_config.md#motionconfig-maxwheelspeed) |
| `MotionConfig::rotationRadius` | field | [motion_config.md](motion_config.md#motionconfig-rotationradius) |
| `MotionConfig::stall` | field | [motion_config.md](motion_config.md#motionconfig-stall) |
| `MotionConfig::translation` | field | [motion_config.md](motion_config.md#motionconfig-translation) |
| `MotionConfig::translationSettle` | field | [motion_config.md](motion_config.md#motionconfig-translationsettle) |
| `MotionConfig::validate` | function | [motion_config.md](motion_config.md#motionconfig-validate) |
| `MotionConfig::wheelFf` | field | [motion_config.md](motion_config.md#motionconfig-wheelff) |
| `MotionDeps` | struct | [motion.md](motion.md#struct-motiondeps) |
| `MotionDeps::ctx` | field | [motion.md](motion.md#motiondeps-ctx) |
| `MotionDeps::faults` | field | [motion.md](motion.md#motiondeps-faults) |
| `MotionDeps::health` | field | [motion.md](motion.md#motiondeps-health) |
| `MotionDeps::kinematics` | field | [motion.md](motion.md#motiondeps-kinematics) |
| `MotionDeps::localizer` | field | [motion.md](motion.md#motiondeps-localizer) |
| `MotionDeps::validate` | function | [motion.md](motion.md#motiondeps-validate) |
| `MotionDeps::validatedClock` | function | [motion.md](motion.md#motiondeps-validatedclock) |
| `MotionOptions` | struct | [chassis.md](chassis.md#struct-motionoptions) |
| `MotionOptions::maxAngularSpeed` | field | [chassis.md](chassis.md#motionoptions-maxangularspeed) |
| `MotionOptions::maxLinearSpeed` | field | [chassis.md](chassis.md#motionoptions-maxlinearspeed) |
| `MotionOptions::timeout` | field | [chassis.md](chassis.md#motionoptions-timeout) |
| `MotionOptions::validate` | function | [chassis.md](chassis.md#motionoptions-validate) |
| `MotionOutcome` | enum class | [motion_result.md](motion_result.md#enum-class-motionoutcome) |
| `MotionOutcome::Cancelled` | enumerator | [motion_result.md](motion_result.md#motionoutcome-cancelled) |
| `MotionOutcome::FaultAbort` | enumerator | [motion_result.md](motion_result.md#motionoutcome-faultabort) |
| `MotionOutcome::Settled` | enumerator | [motion_result.md](motion_result.md#motionoutcome-settled) |
| `MotionOutcome::Superseded` | enumerator | [motion_result.md](motion_result.md#motionoutcome-superseded) |
| `MotionOutcome::TimedOut` | enumerator | [motion_result.md](motion_result.md#motionoutcome-timedout) |
| `motionOutcomeName` | free function | [motion_result.md](motion_result.md#motionoutcomename) |
| `MotionResult` | struct | [motion_result.md](motion_result.md#struct-motionresult) |
| `MotionResult::abortFault` | field | [motion_result.md](motion_result.md#motionresult-abortfault) |
| `MotionResult::drift` | field | [motion_result.md](motion_result.md#motionresult-drift) |
| `MotionResult::duration` | field | [motion_result.md](motion_result.md#motionresult-duration) |
| `MotionResult::finalPose` | field | [motion_result.md](motion_result.md#motionresult-finalpose) |
| `MotionResult::hasPathData` | field | [motion_result.md](motion_result.md#motionresult-haspathdata) |
| `MotionResult::id` | field | [motion_result.md](motion_result.md#motionresult-id) |
| `MotionResult::name` | field | [motion_result.md](motion_result.md#motionresult-name) |
| `MotionResult::outcome` | field | [motion_result.md](motion_result.md#motionresult-outcome) |
| `MotionResult::overshoot` | field | [motion_result.md](motion_result.md#motionresult-overshoot) |
| `MotionScheduler` | class | [motion_scheduler.md](motion_scheduler.md#class-motionscheduler) |
| `MotionScheduler::activeCommandId` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-activecommandid) |
| `MotionScheduler::async` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-async) |
| `MotionScheduler::attribution` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-attribution) |
| `MotionScheduler::boundaryObserver` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-boundaryobserver) |
| `MotionScheduler::cancel` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-cancel) |
| `MotionScheduler::completedCount` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-completedcount) |
| `MotionScheduler::deps` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-deps) |
| `MotionScheduler::hasActiveMotion` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-hasactivemotion) |
| `MotionScheduler::kMaxStalledPaces` | field | [motion_scheduler.md](motion_scheduler.md#motionscheduler-kmaxstalledpaces) |
| `MotionScheduler::lastCompleted` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-lastcompleted) |
| `MotionScheduler::lastExitReason` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-lastexitreason) |
| `MotionScheduler::loopMonitor` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-loopmonitor) |
| `MotionScheduler::motionsAborted` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-motionsaborted) |
| `MotionScheduler::motionsCancelled` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-motionscancelled) |
| `MotionScheduler::MotionScheduler` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-motionscheduler) |
| `MotionScheduler::MotionScheduler (overload 2)` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-motionscheduler-2) |
| `MotionScheduler::MotionScheduler (overload 3)` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-motionscheduler-3) |
| `MotionScheduler::motionsSettled` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-motionssettled) |
| `MotionScheduler::motionsStarted` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-motionsstarted) |
| `MotionScheduler::motionsTimedOut` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-motionstimedout) |
| `MotionScheduler::operator=` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-operator-eq) |
| `MotionScheduler::operator= (overload 2)` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-operator-eq-2) |
| `MotionScheduler::runFinalHeadingDrift` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-runfinalheadingdrift) |
| `MotionScheduler::runHasHeadingData` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-runhasheadingdata) |
| `MotionScheduler::runMaxHeadingDrift` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-runmaxheadingdrift) |
| `MotionScheduler::setBoundaryObserver` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-setboundaryobserver) |
| `MotionScheduler::tick` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-tick) |
| `MotionScheduler::waitUntil` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-waituntil) |
| `MotionScheduler::waitUntilSettled` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-waituntilsettled) |
| `MotionScheduler::~MotionScheduler` | function | [motion_scheduler.md](motion_scheduler.md#motionscheduler-destructor-motionscheduler) |
| `MotionSchedulerConfig` | struct | [motion_scheduler.md](motion_scheduler.md#struct-motionschedulerconfig) |
| `MotionSchedulerConfig::abortFaultMask` | field | [motion_scheduler.md](motion_scheduler.md#motionschedulerconfig-abortfaultmask) |
| `MotionSchedulerConfig::attributionClock` | field | [motion_scheduler.md](motion_scheduler.md#motionschedulerconfig-attributionclock) |
| `MotionSchedulerConfig::loopMonitor` | field | [motion_scheduler.md](motion_scheduler.md#motionschedulerconfig-loopmonitor) |
| `MotionSchedulerConfig::plausibility` | field | [motion_scheduler.md](motion_scheduler.md#motionschedulerconfig-plausibility) |
| `MotionState` | enum class | [motion.md](motion.md#enum-class-motionstate) |
| `MotionState::Cancelled` | enumerator | [motion.md](motion.md#motionstate-cancelled) |
| `MotionState::Idle` | enumerator | [motion.md](motion.md#motionstate-idle) |
| `MotionState::Running` | enumerator | [motion.md](motion.md#motionstate-running) |
| `MotionState::Settled` | enumerator | [motion.md](motion.md#motionstate-settled) |
| `MotionState::TimedOut` | enumerator | [motion.md](motion.md#motionstate-timedout) |
| `MotionState::WaitingForEstimate` | enumerator | [motion.md](motion.md#motionstate-waitingforestimate) |
| `MotionStatsSink` | class | [motion_scheduler.md](motion_scheduler.md#class-motionstatssink) |
| `MotionStatsSink::beginMotion` | function | [motion_scheduler.md](motion_scheduler.md#motionstatssink-beginmotion) |
| `MotionStatsSink::drift` | function | [motion_scheduler.md](motion_scheduler.md#motionstatssink-drift) |
| `MotionStatsSink::emit` | function | [motion_scheduler.md](motion_scheduler.md#motionstatssink-emit) |
| `MotionStatsSink::hasData` | function | [motion_scheduler.md](motion_scheduler.md#motionstatssink-hasdata) |
| `MotionStatsSink::log` | function | [motion_scheduler.md](motion_scheduler.md#motionstatssink-log) |
| `MotionStatsSink::MotionStatsSink` | function | [motion_scheduler.md](motion_scheduler.md#motionstatssink-motionstatssink) |
| `MotionStatsSink::overshoot` | function | [motion_scheduler.md](motion_scheduler.md#motionstatssink-overshoot) |
| `MotionStatsSink::summarize` | function | [motion_scheduler.md](motion_scheduler.md#motionstatssink-summarize) |
| `MotionStatsSink::targetPose` | function | [motion_scheduler.md](motion_scheduler.md#motionstatssink-targetpose) |
| `MotionStatsSink::wantsRecord` | function | [motion_scheduler.md](motion_scheduler.md#motionstatssink-wantsrecord) |
| `MotorGearset` | enum class | [pros-motor.md](pros-motor.md#enum-class-motorgearset) |
| `MotorGearset::Blue` | enumerator | [pros-motor.md](pros-motor.md#motorgearset-blue) |
| `MotorGearset::Green` | enumerator | [pros-motor.md](pros-motor.md#motorgearset-green) |
| `MotorGearset::Red` | enumerator | [pros-motor.md](pros-motor.md#motorgearset-red) |
| `MotorMechanism` | class | [mechanism.md](mechanism.md#class-motormechanism) |
| `MotorMechanism::applySafeState` | function | [mechanism.md](mechanism.md#motormechanism-applysafestate) |
| `MotorMechanism::commandedVoltage` | function | [mechanism.md](mechanism.md#motormechanism-commandedvoltage) |
| `MotorMechanism::maxCurrent` | function | [mechanism.md](mechanism.md#motormechanism-maxcurrent) |
| `MotorMechanism::meanVelocity` | function | [mechanism.md](mechanism.md#motormechanism-meanvelocity) |
| `MotorMechanism::MotorMechanism` | function | [mechanism.md](mechanism.md#motormechanism-motormechanism) |
| `MotorMechanism::motors` | function | [mechanism.md](mechanism.md#motormechanism-motors) |
| `MotorMechanism::name` | function | [mechanism.md](mechanism.md#motormechanism-name) |
| `MotorMechanism::safeBrakeMode` | function | [mechanism.md](mechanism.md#motormechanism-safebrakemode) |
| `MotorMechanism::setVoltage` | function | [mechanism.md](mechanism.md#motormechanism-setvoltage) |
| `motorMilliampsToCanonical` | free function | [motor_conversion.md](motor_conversion.md#motormilliampstocanonical) |
| `motorPositionDegToCanonical` | free function | [motor_conversion.md](motor_conversion.md#motorpositiondegtocanonical) |
| `motorRpmToCanonical` | free function | [motor_conversion.md](motor_conversion.md#motorrpmtocanonical) |
| `motorVoltageApplied` | free function | [motor_conversion.md](motor_conversion.md#motorvoltageapplied) |
| `motorVoltageToMillivolts` | free function | [motor_conversion.md](motor_conversion.md#motorvoltagetomillivolts) |
| `MoveToPose` | class | [move_to_pose.md](move_to_pose.md#class-movetopose) |
| `MoveToPose::cancel` | function | [move_to_pose.md](move_to_pose.md#movetopose-cancel) |
| `MoveToPose::exitReason` | function | [move_to_pose.md](move_to_pose.md#movetopose-exitreason) |
| `MoveToPose::MoveToPose` | function | [move_to_pose.md](move_to_pose.md#movetopose-movetopose) |
| `MoveToPose::name` | function | [move_to_pose.md](move_to_pose.md#movetopose-name) |
| `MoveToPose::setTarget` | function | [move_to_pose.md](move_to_pose.md#movetopose-settarget) |
| `MoveToPose::start` | function | [move_to_pose.md](move_to_pose.md#movetopose-start) |
| `MoveToPose::state` | function | [move_to_pose.md](move_to_pose.md#movetopose-state) |
| `MoveToPose::target` | function | [move_to_pose.md](move_to_pose.md#movetopose-target) |
| `MoveToPose::tick` | function | [move_to_pose.md](move_to_pose.md#movetopose-tick) |

## N

| Name | Kind | Page |
|---|---|---|
| `NullCorrector` | class | [i_corrector.md](i_corrector.md#class-nullcorrector) |
| `NullCorrector::name` | function | [i_corrector.md](i_corrector.md#nullcorrector-name) |
| `NullCorrector::propose` | function | [i_corrector.md](i_corrector.md#nullcorrector-propose) |
| `NullSink` | class | [null_sink.md](null_sink.md#class-nullsink) |
| `NullSink::log` | function | [null_sink.md](null_sink.md#nullsink-log) |
| `Number` | type alias | [quantity.md](quantity.md#number) |

## O

| Name | Kind | Page |
|---|---|---|
| `ObjectObservation` | struct | [vision.md](vision.md#struct-objectobservation) |
| `ObjectObservation::bearing` | field | [vision.md](vision.md#objectobservation-bearing) |
| `ObjectObservation::classId` | field | [vision.md](vision.md#objectobservation-classid) |
| `ObjectObservation::confidence` | field | [vision.md](vision.md#objectobservation-confidence) |
| `OdoStallCheck` | class | [odo_stall_check.md](odo_stall_check.md#class-odostallcheck) |
| `OdoStallCheck::kMaxWheels` | field | [odo_stall_check.md](odo_stall_check.md#odostallcheck-kmaxwheels) |
| `OdoStallCheck::OdoStallCheck` | function | [odo_stall_check.md](odo_stall_check.md#odostallcheck-odostallcheck) |
| `OdoStallCheck::reset` | function | [odo_stall_check.md](odo_stall_check.md#odostallcheck-reset) |
| `OdoStallCheck::stalled` | function | [odo_stall_check.md](odo_stall_check.md#odostallcheck-stalled) |
| `OdoStallCheck::update` | function | [odo_stall_check.md](odo_stall_check.md#odostallcheck-update) |
| `OdoStallCheckConfig` | struct | [odo_stall_check.md](odo_stall_check.md#struct-odostallcheckconfig) |
| `OdoStallCheckConfig::minSpinTravel` | field | [odo_stall_check.md](odo_stall_check.md#odostallcheckconfig-minspintravel) |
| `OdoStallCheckConfig::motionRatio` | field | [odo_stall_check.md](odo_stall_check.md#odostallcheckconfig-motionratio) |
| `OdoStallCheckConfig::rotationRadius` | field | [odo_stall_check.md](odo_stall_check.md#odostallcheckconfig-rotationradius) |
| `OdoStallCheckConfig::wheelRadius` | field | [odo_stall_check.md](odo_stall_check.md#odostallcheckconfig-wheelradius) |
| `OdoStallCheckConfig::window` | field | [odo_stall_check.md](odo_stall_check.md#odostallcheckconfig-window) |
| `operator""_deg` | free function | [literals.md](literals.md#operator-quote-quote-_deg) |
| `operator""_deg (overload 2)` | free function | [literals.md](literals.md#operator-quote-quote-_deg-2) |
| `operator""_in` | free function | [literals.md](literals.md#operator-quote-quote-_in) |
| `operator""_in (overload 2)` | free function | [literals.md](literals.md#operator-quote-quote-_in-2) |
| `operator""_ms` | free function | [literals.md](literals.md#operator-quote-quote-_ms) |
| `operator""_ms (overload 2)` | free function | [literals.md](literals.md#operator-quote-quote-_ms-2) |
| `operator""_rad` | free function | [literals.md](literals.md#operator-quote-quote-_rad) |
| `operator""_rad (overload 2)` | free function | [literals.md](literals.md#operator-quote-quote-_rad-2) |
| `operator""_s` | free function | [literals.md](literals.md#operator-quote-quote-_s) |
| `operator""_s (overload 2)` | free function | [literals.md](literals.md#operator-quote-quote-_s-2) |
| `operator""_tile` | free function | [literals.md](literals.md#operator-quote-quote-_tile) |
| `operator""_tile (overload 2)` | free function | [literals.md](literals.md#operator-quote-quote-_tile-2) |
| `operator""_volt` | free function | [literals.md](literals.md#operator-quote-quote-_volt) |
| `operator""_volt (overload 2)` | free function | [literals.md](literals.md#operator-quote-quote-_volt-2) |
| `operator*` | free function | [quantity.md](quantity.md#operator-star) |
| `operator/` | free function | [quantity.md](quantity.md#operator-slash) |
| `opticalHueToCanonical` | free function | [optical_conversion.md](optical_conversion.md#opticalhuetocanonical) |
| `opticalProximityToCanonical` | free function | [optical_conversion.md](optical_conversion.md#opticalproximitytocanonical) |
| `opticalUnitIntervalToCanonical` | free function | [optical_conversion.md](optical_conversion.md#opticalunitintervaltocanonical) |

## P

| Name | Kind | Page |
|---|---|---|
| `Pid` | class | [pid.md](pid.md#class-pid) |
| `Pid::integralAccumulator` | function | [pid.md](pid.md#pid-integralaccumulator) |
| `Pid::lastError` | function | [pid.md](pid.md#pid-lasterror) |
| `Pid::Pid` | function | [pid.md](pid.md#pid-pid) |
| `Pid::reset` | function | [pid.md](pid.md#pid-reset) |
| `Pid::update` | function | [pid.md](pid.md#pid-update) |
| `PidConfig` | struct | [pid.md](pid.md#struct-pidconfig) |
| `PidConfig::integralLimit` | field | [pid.md](pid.md#pidconfig-integrallimit) |
| `PidConfig::kD` | field | [pid.md](pid.md#pidconfig-kd) |
| `PidConfig::kI` | field | [pid.md](pid.md#pidconfig-ki) |
| `PidConfig::kP` | field | [pid.md](pid.md#pidconfig-kp) |
| `PidConfig::outputMax` | field | [pid.md](pid.md#pidconfig-outputmax) |
| `PidConfig::outputMin` | field | [pid.md](pid.md#pidconfig-outputmin) |
| `PilonsOdometry` | class | [pilons_odometry.md](pilons_odometry.md#class-pilonsodometry) |
| `PilonsOdometry::lastDeltaImplausible` | function | [pilons_odometry.md](pilons_odometry.md#pilonsodometry-lastdeltaimplausible) |
| `PilonsOdometry::PilonsOdometry` | function | [pilons_odometry.md](pilons_odometry.md#pilonsodometry-pilonsodometry) |
| `PilonsOdometry::pose` | function | [pilons_odometry.md](pilons_odometry.md#pilonsodometry-pose) |
| `PilonsOdometry::setPose` | function | [pilons_odometry.md](pilons_odometry.md#pilonsodometry-setpose) |
| `PilonsOdometry::update` | function | [pilons_odometry.md](pilons_odometry.md#pilonsodometry-update) |
| `PilonsOdometryConfig` | struct | [pilons_odometry.md](pilons_odometry.md#struct-pilonsodometryconfig) |
| `PilonsOdometryConfig::maxTickRotation` | field | [pilons_odometry.md](pilons_odometry.md#pilonsodometryconfig-maxtickrotation) |
| `PlausibilityConfig` | struct | [plausibility_guard.md](plausibility_guard.md#struct-plausibilityconfig) |
| `PlausibilityConfig::margin` | field | [plausibility_guard.md](plausibility_guard.md#plausibilityconfig-margin) |
| `PlausibilityConfig::maxSpeed` | field | [plausibility_guard.md](plausibility_guard.md#plausibilityconfig-maxspeed) |
| `PlausibilityConfig::maxYawRate` | field | [plausibility_guard.md](plausibility_guard.md#plausibilityconfig-maxyawrate) |
| `PlausibilityConfig::validate` | function | [plausibility_guard.md](plausibility_guard.md#plausibilityconfig-validate) |
| `PneumaticMechanism` | class | [mechanism.md](mechanism.md#class-pneumaticmechanism) |
| `PneumaticMechanism::applySafeState` | function | [mechanism.md](mechanism.md#pneumaticmechanism-applysafestate) |
| `PneumaticMechanism::commanded` | function | [mechanism.md](mechanism.md#pneumaticmechanism-commanded) |
| `PneumaticMechanism::lines` | function | [mechanism.md](mechanism.md#pneumaticmechanism-lines) |
| `PneumaticMechanism::name` | function | [mechanism.md](mechanism.md#pneumaticmechanism-name) |
| `PneumaticMechanism::PneumaticMechanism` | function | [mechanism.md](mechanism.md#pneumaticmechanism-pneumaticmechanism) |
| `PneumaticMechanism::safeCommand` | function | [mechanism.md](mechanism.md#pneumaticmechanism-safecommand) |
| `PneumaticMechanism::set` | function | [mechanism.md](mechanism.md#pneumaticmechanism-set) |
| `Pose2d` | class | [pose2d.md](pose2d.md#class-pose2d) |
| `Pose2d::approxEqual` | function | [pose2d.md](pose2d.md#pose2d-approxequal) |
| `Pose2d::heading` | function | [pose2d.md](pose2d.md#pose2d-heading) |
| `Pose2d::Pose2d` | function | [pose2d.md](pose2d.md#pose2d-pose2d) |
| `Pose2d::Pose2d (overload 2)` | function | [pose2d.md](pose2d.md#pose2d-pose2d-2) |
| `Pose2d::x` | function | [pose2d.md](pose2d.md#pose2d-x) |
| `Pose2d::y` | function | [pose2d.md](pose2d.md#pose2d-y) |
| `PoseDeltaGuard` | class | [plausibility_guard.md](plausibility_guard.md#class-posedeltaguard) |
| `PoseDeltaGuard::check` | function | [plausibility_guard.md](plausibility_guard.md#posedeltaguard-check) |
| `PoseDeltaGuard::PoseDeltaGuard` | function | [plausibility_guard.md](plausibility_guard.md#posedeltaguard-posedeltaguard) |
| `PoseDeltaGuard::reset` | function | [plausibility_guard.md](plausibility_guard.md#posedeltaguard-reset) |
| `PoseMotionOptions` | struct | [move_to_pose.md](move_to_pose.md#struct-posemotionoptions) |
| `PoseMotionOptions::captureHeadingAtLive` | field | [move_to_pose.md](move_to_pose.md#posemotionoptions-captureheadingatlive) |
| `PoseMotionOptions::capturePoseAtLive` | field | [move_to_pose.md](move_to_pose.md#posemotionoptions-captureposeatlive) |
| `PoseMotionOptions::holdFor` | field | [move_to_pose.md](move_to_pose.md#posemotionoptions-holdfor) |
| `Power` | type alias | [quantity.md](quantity.md#power) |
| `precondition_failed` | free function | [check.md](check.md#precondition_failed) |
| `PreconditionError` | struct | [check.md](check.md#struct-preconditionerror) |
| `PreconditionError::logic_error` | alias | [check.md](check.md#preconditionerror-logic_error) |
| `PreconditionHandler` | type alias | [check.md](check.md#preconditionhandler) |
| `preconditionHandler (overload 2)` | free function | [check.md](check.md#preconditionhandler-2) |
| `ProfileConstraints` | struct | [trapezoid_profile.md](trapezoid_profile.md#struct-profileconstraints) |
| `ProfileConstraints::maxAcceleration` | field | [trapezoid_profile.md](trapezoid_profile.md#profileconstraints-maxacceleration) |
| `ProfileConstraints::maxVelocity` | field | [trapezoid_profile.md](trapezoid_profile.md#profileconstraints-maxvelocity) |
| `ProfileState` | struct | [trapezoid_profile.md](trapezoid_profile.md#struct-profilestate) |
| `ProfileState::acceleration` | field | [trapezoid_profile.md](trapezoid_profile.md#profilestate-acceleration) |
| `ProfileState::position` | field | [trapezoid_profile.md](trapezoid_profile.md#profilestate-position) |
| `ProfileState::velocity` | field | [trapezoid_profile.md](trapezoid_profile.md#profilestate-velocity) |
| `ProsBattery` | class | [pros-battery.md](pros-battery.md#class-prosbattery) |
| `ProsBattery::capacity` | function | [pros-battery.md](pros-battery.md#prosbattery-capacity) |
| `ProsBattery::current` | function | [pros-battery.md](pros-battery.md#prosbattery-current) |
| `ProsBattery::faultedReads` | function | [pros-battery.md](pros-battery.md#prosbattery-faultedreads) |
| `ProsBattery::voltage` | function | [pros-battery.md](pros-battery.md#prosbattery-voltage) |
| `ProsBlockSink` | class | [pros-block_sink.md](pros-block_sink.md#class-prosblocksink) |
| `ProsBlockSink::flush` | function | [pros-block_sink.md](pros-block_sink.md#prosblocksink-flush) |
| `ProsBlockSink::isOpen` | function | [pros-block_sink.md](pros-block_sink.md#prosblocksink-isopen) |
| `ProsBlockSink::operator=` | function | [pros-block_sink.md](pros-block_sink.md#prosblocksink-operator-eq) |
| `ProsBlockSink::operator= (overload 2)` | function | [pros-block_sink.md](pros-block_sink.md#prosblocksink-operator-eq-2) |
| `ProsBlockSink::path` | function | [pros-block_sink.md](pros-block_sink.md#prosblocksink-path) |
| `ProsBlockSink::ProsBlockSink` | function | [pros-block_sink.md](pros-block_sink.md#prosblocksink-prosblocksink) |
| `ProsBlockSink::ProsBlockSink (overload 2)` | function | [pros-block_sink.md](pros-block_sink.md#prosblocksink-prosblocksink-2) |
| `ProsBlockSink::ProsBlockSink (overload 3)` | function | [pros-block_sink.md](pros-block_sink.md#prosblocksink-prosblocksink-3) |
| `ProsBlockSink::write` | function | [pros-block_sink.md](pros-block_sink.md#prosblocksink-write) |
| `ProsBlockSink::~ProsBlockSink` | function | [pros-block_sink.md](pros-block_sink.md#prosblocksink-destructor-prosblocksink) |
| `ProsCharSink` | class | [pros-char_sink.md](pros-char_sink.md#class-proscharsink) |
| `ProsCharSink::ProsCharSink` | function | [pros-char_sink.md](pros-char_sink.md#proscharsink-proscharsink) |
| `ProsCharSink::write` | function | [pros-char_sink.md](pros-char_sink.md#proscharsink-write) |
| `ProsClock` | class | [pros-clock.md](pros-clock.md#class-prosclock) |
| `ProsClock::now` | function | [pros-clock.md](pros-clock.md#prosclock-now) |
| `ProsController` | class | [pros-controller.md](pros-controller.md#class-proscontroller) |
| `ProsController::axis` | function | [pros-controller.md](pros-controller.md#proscontroller-axis) |
| `ProsController::isConnected` | function | [pros-controller.md](pros-controller.md#proscontroller-isconnected) |
| `ProsController::pressed` | function | [pros-controller.md](pros-controller.md#proscontroller-pressed) |
| `ProsController::ProsController` | function | [pros-controller.md](pros-controller.md#proscontroller-proscontroller) |
| `ProsDigitalIn` | class | [pros-digital_in.md](pros-digital_in.md#class-prosdigitalin) |
| `ProsDigitalIn::faultedReads` | function | [pros-digital_in.md](pros-digital_in.md#prosdigitalin-faultedreads) |
| `ProsDigitalIn::ProsDigitalIn` | function | [pros-digital_in.md](pros-digital_in.md#prosdigitalin-prosdigitalin) |
| `ProsDigitalIn::ProsDigitalIn (overload 2)` | function | [pros-digital_in.md](pros-digital_in.md#prosdigitalin-prosdigitalin-2) |
| `ProsDigitalIn::state` | function | [pros-digital_in.md](pros-digital_in.md#prosdigitalin-state) |
| `ProsDigitalOut` | class | [pros-digital_out.md](pros-digital_out.md#class-prosdigitalout) |
| `ProsDigitalOut::commanded` | function | [pros-digital_out.md](pros-digital_out.md#prosdigitalout-commanded) |
| `ProsDigitalOut::faultedWrites` | function | [pros-digital_out.md](pros-digital_out.md#prosdigitalout-faultedwrites) |
| `ProsDigitalOut::ProsDigitalOut` | function | [pros-digital_out.md](pros-digital_out.md#prosdigitalout-prosdigitalout) |
| `ProsDigitalOut::ProsDigitalOut (overload 2)` | function | [pros-digital_out.md](pros-digital_out.md#prosdigitalout-prosdigitalout-2) |
| `ProsDigitalOut::set` | function | [pros-digital_out.md](pros-digital_out.md#prosdigitalout-set) |
| `ProsDistance` | class | [pros-distance.md](pros-distance.md#class-prosdistance) |
| `ProsDistance::confidence` | function | [pros-distance.md](pros-distance.md#prosdistance-confidence) |
| `ProsDistance::distance` | function | [pros-distance.md](pros-distance.md#prosdistance-distance) |
| `ProsDistance::faultedReads` | function | [pros-distance.md](pros-distance.md#prosdistance-faultedreads) |
| `ProsDistance::ProsDistance` | function | [pros-distance.md](pros-distance.md#prosdistance-prosdistance) |
| `ProsGps` | class | [pros-gps.md](pros-gps.md#class-prosgps) |
| `ProsGps::faultedReads` | function | [pros-gps.md](pros-gps.md#prosgps-faultedreads) |
| `ProsGps::hasFix` | function | [pros-gps.md](pros-gps.md#prosgps-hasfix) |
| `ProsGps::pose` | function | [pros-gps.md](pros-gps.md#prosgps-pose) |
| `ProsGps::ProsGps` | function | [pros-gps.md](pros-gps.md#prosgps-prosgps) |
| `ProsGps::rmsError` | function | [pros-gps.md](pros-gps.md#prosgps-rmserror) |
| `ProsImu` | class | [pros-imu.md](pros-imu.md#class-prosimu) |
| `ProsImu::calibrate` | function | [pros-imu.md](pros-imu.md#prosimu-calibrate) |
| `ProsImu::faultedReads` | function | [pros-imu.md](pros-imu.md#prosimu-faultedreads) |
| `ProsImu::heading` | function | [pros-imu.md](pros-imu.md#prosimu-heading) |
| `ProsImu::isReady` | function | [pros-imu.md](pros-imu.md#prosimu-isready) |
| `ProsImu::pitch` | function | [pros-imu.md](pros-imu.md#prosimu-pitch) |
| `ProsImu::ProsImu` | function | [pros-imu.md](pros-imu.md#prosimu-prosimu) |
| `ProsImu::roll` | function | [pros-imu.md](pros-imu.md#prosimu-roll) |
| `ProsImu::yawRate` | function | [pros-imu.md](pros-imu.md#prosimu-yawrate) |
| `ProsLineDisplay` | class | [pros-line_display.md](pros-line_display.md#class-proslinedisplay) |
| `ProsLineDisplay::ProsLineDisplay` | function | [pros-line_display.md](pros-line_display.md#proslinedisplay-proslinedisplay) |
| `ProsLineDisplay::setLine` | function | [pros-line_display.md](pros-line_display.md#proslinedisplay-setline) |
| `ProsMotor` | class | [pros-motor.md](pros-motor.md#class-prosmotor) |
| `ProsMotor::brakeMode` | function | [pros-motor.md](pros-motor.md#prosmotor-brakemode) |
| `ProsMotor::commandedVoltage` | function | [pros-motor.md](pros-motor.md#prosmotor-commandedvoltage) |
| `ProsMotor::current` | function | [pros-motor.md](pros-motor.md#prosmotor-current) |
| `ProsMotor::faultedReads` | function | [pros-motor.md](pros-motor.md#prosmotor-faultedreads) |
| `ProsMotor::position` | function | [pros-motor.md](pros-motor.md#prosmotor-position) |
| `ProsMotor::ProsMotor` | function | [pros-motor.md](pros-motor.md#prosmotor-prosmotor) |
| `ProsMotor::setBrakeMode` | function | [pros-motor.md](pros-motor.md#prosmotor-setbrakemode) |
| `ProsMotor::setVoltage` | function | [pros-motor.md](pros-motor.md#prosmotor-setvoltage) |
| `ProsMotor::temperature` | function | [pros-motor.md](pros-motor.md#prosmotor-temperature) |
| `ProsMotor::velocity` | function | [pros-motor.md](pros-motor.md#prosmotor-velocity) |
| `ProsOptical` | class | [pros-optical.md](pros-optical.md#class-prosoptical) |
| `ProsOptical::brightness` | function | [pros-optical.md](pros-optical.md#prosoptical-brightness) |
| `ProsOptical::faultedReads` | function | [pros-optical.md](pros-optical.md#prosoptical-faultedreads) |
| `ProsOptical::hue` | function | [pros-optical.md](pros-optical.md#prosoptical-hue) |
| `ProsOptical::ProsOptical` | function | [pros-optical.md](pros-optical.md#prosoptical-prosoptical) |
| `ProsOptical::proximity` | function | [pros-optical.md](pros-optical.md#prosoptical-proximity) |
| `ProsOptical::saturation` | function | [pros-optical.md](pros-optical.md#prosoptical-saturation) |
| `ProsRotation` | class | [pros-rotation.md](pros-rotation.md#class-prosrotation) |
| `ProsRotation::faultedReads` | function | [pros-rotation.md](pros-rotation.md#prosrotation-faultedreads) |
| `ProsRotation::position` | function | [pros-rotation.md](pros-rotation.md#prosrotation-position) |
| `ProsRotation::ProsRotation` | function | [pros-rotation.md](pros-rotation.md#prosrotation-prosrotation) |
| `ProsRotation::velocity` | function | [pros-rotation.md](pros-rotation.md#prosrotation-velocity) |
| `ProsTickPacer` | class | [pros-tick_pacer.md](pros-tick_pacer.md#class-prostickpacer) |
| `ProsTickPacer::kTickMs` | field | [pros-tick_pacer.md](pros-tick_pacer.md#prostickpacer-ktickms) |
| `ProsTickPacer::pace` | function | [pros-tick_pacer.md](pros-tick_pacer.md#prostickpacer-pace) |

## Q

| Name | Kind | Page |
|---|---|---|
| `Quantity` | class | [quantity.md](quantity.md#class-quantity) |
| `Quantity::operator*` | function | [quantity.md](quantity.md#quantity-operator-star) |
| `Quantity::operator* (overload 2)` | function | [quantity.md](quantity.md#quantity-operator-star-2) |
| `Quantity::operator+` | function | [quantity.md](quantity.md#quantity-operator-plus) |
| `Quantity::operator+=` | function | [quantity.md](quantity.md#quantity-operator-plus-eq) |
| `Quantity::operator-` | function | [quantity.md](quantity.md#quantity-operator-minus) |
| `Quantity::operator- (overload 2)` | function | [quantity.md](quantity.md#quantity-operator-minus-2) |
| `Quantity::operator-=` | function | [quantity.md](quantity.md#quantity-operator-minus-eq) |
| `Quantity::operator/` | function | [quantity.md](quantity.md#quantity-operator-slash) |
| `Quantity::operator<=>` | function | [quantity.md](quantity.md#quantity-operator-lt-eq-gt) |
| `Quantity::operator==` | function | [quantity.md](quantity.md#quantity-operator-eq-eq) |
| `Quantity::Quantity` | function | [quantity.md](quantity.md#quantity-quantity) |
| `Quantity::Quantity (overload 2)` | function | [quantity.md](quantity.md#quantity-quantity-2) |
| `Quantity::value` | function | [quantity.md](quantity.md#quantity-value) |

## R

| Name | Kind | Page |
|---|---|---|
| `RateLimitConfig` | struct | [rate_limit_sink.md](rate_limit_sink.md#struct-ratelimitconfig) |
| `RateLimitConfig::linesPerSecondPerChannel` | field | [rate_limit_sink.md](rate_limit_sink.md#ratelimitconfig-linespersecondperchannel) |
| `RateLimitConfig::recordsPerSecond` | field | [rate_limit_sink.md](rate_limit_sink.md#ratelimitconfig-recordspersecond) |
| `RateLimitedSink` | class | [rate_limit_sink.md](rate_limit_sink.md#class-ratelimitedsink) |
| `RateLimitedSink::droppedLines` | function | [rate_limit_sink.md](rate_limit_sink.md#ratelimitedsink-droppedlines) |
| `RateLimitedSink::droppedRecords` | function | [rate_limit_sink.md](rate_limit_sink.md#ratelimitedsink-droppedrecords) |
| `RateLimitedSink::emit` | function | [rate_limit_sink.md](rate_limit_sink.md#ratelimitedsink-emit) |
| `RateLimitedSink::log` | function | [rate_limit_sink.md](rate_limit_sink.md#ratelimitedsink-log) |
| `RateLimitedSink::RateLimitedSink` | function | [rate_limit_sink.md](rate_limit_sink.md#ratelimitedsink-ratelimitedsink) |
| `RateLimitedSink::summarize` | function | [rate_limit_sink.md](rate_limit_sink.md#ratelimitedsink-summarize) |
| `RateLimitedSink::wantsRecord` | function | [rate_limit_sink.md](rate_limit_sink.md#ratelimitedsink-wantsrecord) |
| `ReadStatus` | enum class | [blackbox_reader.md](blackbox_reader.md#enum-class-readstatus) |
| `ReadStatus::BadMagic` | enumerator | [blackbox_reader.md](blackbox_reader.md#readstatus-badmagic) |
| `ReadStatus::Empty` | enumerator | [blackbox_reader.md](blackbox_reader.md#readstatus-empty) |
| `ReadStatus::HeaderTruncated` | enumerator | [blackbox_reader.md](blackbox_reader.md#readstatus-headertruncated) |
| `ReadStatus::LayoutMismatch` | enumerator | [blackbox_reader.md](blackbox_reader.md#readstatus-layoutmismatch) |
| `ReadStatus::Ok` | enumerator | [blackbox_reader.md](blackbox_reader.md#readstatus-ok) |
| `ReadStatus::UnsupportedVersion` | enumerator | [blackbox_reader.md](blackbox_reader.md#readstatus-unsupportedversion) |
| `readStatusName` | free function | [blackbox_reader.md](blackbox_reader.md#readstatusname) |
| `recoverFinite` | free function | [finite_guard.md](finite_guard.md#recoverfinite) |
| `recoverFinitePose` | free function | [finite_guard.md](finite_guard.md#recoverfinitepose) |
| `recoverWheelVoltage` | free function | [plausibility_guard.md](plausibility_guard.md#recoverwheelvoltage) |
| `RobotContext` | class | [robot_context.md](robot_context.md#class-robotcontext) |
| `RobotContext::battery` | function | [robot_context.md](robot_context.md#robotcontext-battery) |
| `RobotContext::clock` | function | [robot_context.md](robot_context.md#robotcontext-clock) |
| `RobotContext::driveMotors` | function | [robot_context.md](robot_context.md#robotcontext-drivemotors) |
| `RobotContext::gps` | function | [robot_context.md](robot_context.md#robotcontext-gps) |
| `RobotContext::imu` | function | [robot_context.md](robot_context.md#robotcontext-imu) |
| `RobotContext::RobotContext` | function | [robot_context.md](robot_context.md#robotcontext-robotcontext) |
| `RobotContext::tags` | function | [robot_context.md](robot_context.md#robotcontext-tags) |
| `RobotContext::telemetry` | function | [robot_context.md](robot_context.md#robotcontext-telemetry) |
| `RobotContext::vision` | function | [robot_context.md](robot_context.md#robotcontext-vision) |
| `RobotContextConfig` | struct | [robot_context.md](robot_context.md#struct-robotcontextconfig) |
| `RobotContextConfig::battery` | field | [robot_context.md](robot_context.md#robotcontextconfig-battery) |
| `RobotContextConfig::clock` | field | [robot_context.md](robot_context.md#robotcontextconfig-clock) |
| `RobotContextConfig::driveMotors` | field | [robot_context.md](robot_context.md#robotcontextconfig-drivemotors) |
| `RobotContextConfig::gps` | field | [robot_context.md](robot_context.md#robotcontextconfig-gps) |
| `RobotContextConfig::imu` | field | [robot_context.md](robot_context.md#robotcontextconfig-imu) |
| `RobotContextConfig::tags` | field | [robot_context.md](robot_context.md#robotcontextconfig-tags) |
| `RobotContextConfig::telemetry` | field | [robot_context.md](robot_context.md#robotcontextconfig-telemetry) |
| `RobotContextConfig::vision` | field | [robot_context.md](robot_context.md#robotcontextconfig-vision) |
| `robotToField` | free function | [frame.md](frame.md#robottofield) |
| `rotationCentidegPerSecToCanonical` | free function | [rotation_conversion.md](rotation_conversion.md#rotationcentidegpersectocanonical) |
| `rotationCentidegToCanonical` | free function | [rotation_conversion.md](rotation_conversion.md#rotationcentidegtocanonical) |
| `Routine` | class | [routine.md](routine.md#class-routine) |
| `Routine::brake` | function | [routine.md](routine.md#routine-brake) |
| `Routine::chassis` | function | [routine.md](routine.md#routine-chassis) |
| `Routine::driveTo` | function | [routine.md](routine.md#routine-driveto) |
| `Routine::face` | function | [routine.md](routine.md#routine-face) |
| `Routine::followTrajectory` | function | [routine.md](routine.md#routine-followtrajectory) |
| `Routine::followTrajectory (overload 2)` | function | [routine.md](routine.md#routine-followtrajectory-2) |
| `Routine::hold` | function | [routine.md](routine.md#routine-hold) |
| `Routine::lastTrajectory` | function | [routine.md](routine.md#routine-lasttrajectory) |
| `Routine::moveTo` | function | [routine.md](routine.md#routine-moveto) |
| `Routine::ok` | function | [routine.md](routine.md#routine-ok) |
| `Routine::operator=` | function | [routine.md](routine.md#routine-operator-eq) |
| `Routine::operator= (overload 2)` | function | [routine.md](routine.md#routine-operator-eq-2) |
| `Routine::pause` | function | [routine.md](routine.md#routine-pause) |
| `Routine::result` | function | [routine.md](routine.md#routine-result) |
| `Routine::Routine` | function | [routine.md](routine.md#routine-routine) |
| `Routine::Routine (overload 2)` | function | [routine.md](routine.md#routine-routine-2) |
| `Routine::Routine (overload 3)` | function | [routine.md](routine.md#routine-routine-3) |
| `Routine::startAt` | function | [routine.md](routine.md#routine-startat) |
| `Routine::strafeTo` | function | [routine.md](routine.md#routine-strafeto) |
| `Routine::then` | function | [routine.md](routine.md#routine-then) |
| `Routine::turnTo` | function | [routine.md](routine.md#routine-turnto) |
| `Routine::waitFor` | function | [routine.md](routine.md#routine-waitfor) |
| `Routine::~Routine` | function | [routine.md](routine.md#routine-destructor-routine) |
| `RoutineResult` | struct | [routine.md](routine.md#struct-routineresult) |
| `RoutineResult::cause` | field | [routine.md](routine.md#routineresult-cause) |
| `RoutineResult::completed` | field | [routine.md](routine.md#routineresult-completed) |
| `RoutineResult::exit` | field | [routine.md](routine.md#routineresult-exit) |
| `RoutineResult::ok` | field | [routine.md](routine.md#routineresult-ok) |
| `RoutineResult::skipped` | field | [routine.md](routine.md#routineresult-skipped) |
| `RoutineResult::steps` | field | [routine.md](routine.md#routineresult-steps) |
| `RoutineResult::stoppedAt` | field | [routine.md](routine.md#routineresult-stoppedat) |
| `RoutineResult::stoppedName` | field | [routine.md](routine.md#routineresult-stoppedname) |
| `RoutineStopCause` | enum class | [routine.md](routine.md#enum-class-routinestopcause) |
| `RoutineStopCause::ActionFailed` | enumerator | [routine.md](routine.md#routinestopcause-actionfailed) |
| `RoutineStopCause::MechanismFailed` | enumerator | [routine.md](routine.md#routinestopcause-mechanismfailed) |
| `RoutineStopCause::MotionFailed` | enumerator | [routine.md](routine.md#routinestopcause-motionfailed) |
| `RoutineStopCause::None` | enumerator | [routine.md](routine.md#routinestopcause-none) |
| `RoutineStopCause::WaitTimedOut` | enumerator | [routine.md](routine.md#routinestopcause-waittimedout) |
| `RunGuard` | class | [run_guard.md](run_guard.md#class-runguard) |
| `RunGuard::expired` | function | [run_guard.md](run_guard.md#runguard-expired) |
| `RunGuard::operator=` | function | [run_guard.md](run_guard.md#runguard-operator-eq) |
| `RunGuard::operator= (overload 2)` | function | [run_guard.md](run_guard.md#runguard-operator-eq-2) |
| `RunGuard::pace` | function | [run_guard.md](run_guard.md#runguard-pace) |
| `RunGuard::pause` | function | [run_guard.md](run_guard.md#runguard-pause) |
| `RunGuard::remaining` | function | [run_guard.md](run_guard.md#runguard-remaining) |
| `RunGuard::run` | function | [run_guard.md](run_guard.md#runguard-run) |
| `RunGuard::RunGuard` | function | [run_guard.md](run_guard.md#runguard-runguard) |
| `RunGuard::RunGuard (overload 2)` | function | [run_guard.md](run_guard.md#runguard-runguard-2) |
| `RunGuard::RunGuard (overload 3)` | function | [run_guard.md](run_guard.md#runguard-runguard-3) |
| `RunGuard::running` | function | [run_guard.md](run_guard.md#runguard-running) |
| `RunGuard::waitFor` | function | [run_guard.md](run_guard.md#runguard-waitfor) |
| `RunGuard::~RunGuard` | function | [run_guard.md](run_guard.md#runguard-destructor-runguard) |
| `RunGuardConfig` | struct | [run_guard.md](run_guard.md#struct-runguardconfig) |
| `RunGuardConfig::endActionAt` | field | [run_guard.md](run_guard.md#runguardconfig-endactionat) |
| `RunGuardConfig::hardStopAt` | field | [run_guard.md](run_guard.md#runguardconfig-hardstopat) |
| `RunGuardConfig::mechanisms` | field | [run_guard.md](run_guard.md#runguardconfig-mechanisms) |
| `RunGuardConfig::validate` | function | [run_guard.md](run_guard.md#runguardconfig-validate) |
| `RunGuardReport` | struct | [run_guard.md](run_guard.md#struct-runguardreport) |
| `RunGuardReport::anonymousClaimsReleased` | field | [run_guard.md](run_guard.md#runguardreport-anonymousclaimsreleased) |
| `RunGuardReport::endActionEnded` | field | [run_guard.md](run_guard.md#runguardreport-endactionended) |
| `RunGuardReport::endActionRan` | field | [run_guard.md](run_guard.md#runguardreport-endactionran) |
| `RunGuardReport::endActionSucceeded` | field | [run_guard.md](run_guard.md#runguardreport-endactionsucceeded) |
| `RunGuardReport::floorFired` | field | [run_guard.md](run_guard.md#runguardreport-floorfired) |
| `RunGuardReport::pacesSeen` | field | [run_guard.md](run_guard.md#runguardreport-pacesseen) |
| `RunGuardReport::postExpiryCancels` | field | [run_guard.md](run_guard.md#runguardreport-postexpirycancels) |
| `RunGuardReport::scoringCut` | field | [run_guard.md](run_guard.md#runguardreport-scoringcut) |
| `RunGuardReport::scoringEnded` | field | [run_guard.md](run_guard.md#runguardreport-scoringended) |
| `RunReporter` | class | [run_reporter.md](run_reporter.md#class-runreporter) |
| `RunReporter::finishRun` | function | [run_reporter.md](run_reporter.md#runreporter-finishrun) |
| `RunReporter::onMotionComplete` | function | [run_reporter.md](run_reporter.md#runreporter-onmotioncomplete) |
| `RunReporter::operator=` | function | [run_reporter.md](run_reporter.md#runreporter-operator-eq) |
| `RunReporter::operator= (overload 2)` | function | [run_reporter.md](run_reporter.md#runreporter-operator-eq-2) |
| `RunReporter::RunReporter` | function | [run_reporter.md](run_reporter.md#runreporter-runreporter) |
| `RunReporter::RunReporter (overload 2)` | function | [run_reporter.md](run_reporter.md#runreporter-runreporter-2) |
| `RunReporter::RunReporter (overload 3)` | function | [run_reporter.md](run_reporter.md#runreporter-runreporter-3) |
| `RunReporter::sessionStart` | function | [run_reporter.md](run_reporter.md#runreporter-sessionstart) |
| `RunReporter::~RunReporter` | function | [run_reporter.md](run_reporter.md#runreporter-destructor-runreporter) |
| `RunSummary` | struct | [run_summary.md](run_summary.md#struct-runsummary) |
| `RunSummary::batteryEnd` | field | [run_summary.md](run_summary.md#runsummary-batteryend) |
| `RunSummary::batteryStart` | field | [run_summary.md](run_summary.md#runsummary-batterystart) |
| `RunSummary::blackboxDropped` | field | [run_summary.md](run_summary.md#runsummary-blackboxdropped) |
| `RunSummary::brownout` | field | [run_summary.md](run_summary.md#runsummary-brownout) |
| `RunSummary::buildHash` | function | [run_summary.md](run_summary.md#runsummary-buildhash) |
| `RunSummary::droppedLines` | field | [run_summary.md](run_summary.md#runsummary-droppedlines) |
| `RunSummary::droppedRecords` | field | [run_summary.md](run_summary.md#runsummary-droppedrecords) |
| `RunSummary::firstFault` | field | [run_summary.md](run_summary.md#runsummary-firstfault) |
| `RunSummary::firstFaultTime` | field | [run_summary.md](run_summary.md#runsummary-firstfaulttime) |
| `RunSummary::gatingRejects` | field | [run_summary.md](run_summary.md#runsummary-gatingrejects) |
| `RunSummary::hasHeadingData` | field | [run_summary.md](run_summary.md#runsummary-hasheadingdata) |
| `RunSummary::headingFinal` | field | [run_summary.md](run_summary.md#runsummary-headingfinal) |
| `RunSummary::headingMax` | field | [run_summary.md](run_summary.md#runsummary-headingmax) |
| `RunSummary::motionsAborted` | field | [run_summary.md](run_summary.md#runsummary-motionsaborted) |
| `RunSummary::motionsCancelled` | field | [run_summary.md](run_summary.md#runsummary-motionscancelled) |
| `RunSummary::motionsSettled` | field | [run_summary.md](run_summary.md#runsummary-motionssettled) |
| `RunSummary::motionsStarted` | field | [run_summary.md](run_summary.md#runsummary-motionsstarted) |
| `RunSummary::motionsTimedOut` | field | [run_summary.md](run_summary.md#runsummary-motionstimedout) |
| `RunSummary::routineId` | function | [run_summary.md](run_summary.md#runsummary-routineid) |
| `RunSummary::setBuildHash` | function | [run_summary.md](run_summary.md#runsummary-setbuildhash) |
| `RunSummary::setRoutineId` | function | [run_summary.md](run_summary.md#runsummary-setroutineid) |
| `RunSummary::worstLoopDt` | field | [run_summary.md](run_summary.md#runsummary-worstloopdt) |
| `RunUntilConfirmed` | class | [mechanism_op.md](mechanism_op.md#class-rununtilconfirmed) |
| `RunUntilConfirmed::cancel` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-cancel) |
| `RunUntilConfirmed::name` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-name) |
| `RunUntilConfirmed::operator=` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-operator-eq) |
| `RunUntilConfirmed::operator= (overload 2)` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-operator-eq-2) |
| `RunUntilConfirmed::outcome` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-outcome) |
| `RunUntilConfirmed::RunUntilConfirmed` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-rununtilconfirmed) |
| `RunUntilConfirmed::RunUntilConfirmed (overload 2)` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-rununtilconfirmed-2) |
| `RunUntilConfirmed::RunUntilConfirmed (overload 3)` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-rununtilconfirmed-3) |
| `RunUntilConfirmed::start` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-start) |
| `RunUntilConfirmed::started` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-started) |
| `RunUntilConfirmed::tick` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-tick) |
| `RunUntilConfirmed::~RunUntilConfirmed` | function | [mechanism_op.md](mechanism_op.md#rununtilconfirmed-destructor-rununtilconfirmed) |
| `RunUntilConfirmedConfig` | struct | [mechanism_op.md](mechanism_op.md#struct-rununtilconfirmedconfig) |
| `RunUntilConfirmedConfig::stall` | field | [mechanism_op.md](mechanism_op.md#rununtilconfirmedconfig-stall) |
| `RunUntilConfirmedConfig::timeout` | field | [mechanism_op.md](mechanism_op.md#rununtilconfirmedconfig-timeout) |
| `RunUntilConfirmedConfig::voltage` | field | [mechanism_op.md](mechanism_op.md#rununtilconfirmedconfig-voltage) |

## S

| Name | Kind | Page |
|---|---|---|
| `safeAngle` | free function | [blackbox_format.md](blackbox_format.md#safeangle) |
| `SdSink` | class | [sd_sink.md](sd_sink.md#class-sdsink) |
| `SdSink::brownout` | function | [sd_sink.md](sd_sink.md#sdsink-brownout) |
| `SdSink::bytesBuffered` | function | [sd_sink.md](sd_sink.md#sdsink-bytesbuffered) |
| `SdSink::bytesWritten` | function | [sd_sink.md](sd_sink.md#sdsink-byteswritten) |
| `SdSink::close` | function | [sd_sink.md](sd_sink.md#sdsink-close) |
| `SdSink::closed` | function | [sd_sink.md](sd_sink.md#sdsink-closed) |
| `SdSink::deviceFailed` | function | [sd_sink.md](sd_sink.md#sdsink-devicefailed) |
| `SdSink::droppedFrames` | function | [sd_sink.md](sd_sink.md#sdsink-droppedframes) |
| `SdSink::dumped` | function | [sd_sink.md](sd_sink.md#sdsink-dumped) |
| `SdSink::emit` | function | [sd_sink.md](sd_sink.md#sdsink-emit) |
| `SdSink::flush` | function | [sd_sink.md](sd_sink.md#sdsink-flush) |
| `SdSink::log` | function | [sd_sink.md](sd_sink.md#sdsink-log) |
| `SdSink::markBrownout` | function | [sd_sink.md](sd_sink.md#sdsink-markbrownout) |
| `SdSink::messagesSeen` | function | [sd_sink.md](sd_sink.md#sdsink-messagesseen) |
| `SdSink::open` | function | [sd_sink.md](sd_sink.md#sdsink-open) |
| `SdSink::recordsSeen` | function | [sd_sink.md](sd_sink.md#sdsink-recordsseen) |
| `SdSink::ringSize` | function | [sd_sink.md](sd_sink.md#sdsink-ringsize) |
| `SdSink::SdSink` | function | [sd_sink.md](sd_sink.md#sdsink-sdsink) |
| `SdSink::summarize` | function | [sd_sink.md](sd_sink.md#sdsink-summarize) |
| `SdSink::tickFrames` | function | [sd_sink.md](sd_sink.md#sdsink-tickframes) |
| `SdSink::triage` | function | [sd_sink.md](sd_sink.md#sdsink-triage) |
| `SdSink::triageTick` | function | [sd_sink.md](sd_sink.md#sdsink-triagetick) |
| `SdSink::triggerDump` | function | [sd_sink.md](sd_sink.md#sdsink-triggerdump) |
| `SdSink::wantsRecord` | function | [sd_sink.md](sd_sink.md#sdsink-wantsrecord) |
| `SdSinkBuffers` | struct | [sd_sink.md](sd_sink.md#struct-sdsinkbuffers) |
| `SdSinkBuffers::buffer` | field | [sd_sink.md](sd_sink.md#sdsinkbuffers-buffer) |
| `SdSinkBuffers::ring` | field | [sd_sink.md](sd_sink.md#sdsinkbuffers-ring) |
| `SdSinkBuffers::view` | function | [sd_sink.md](sd_sink.md#sdsinkbuffers-view) |
| `SdSinkConfig` | struct | [sd_sink.md](sd_sink.md#struct-sdsinkconfig) |
| `SdSinkConfig::dumpOnFault` | field | [sd_sink.md](sd_sink.md#sdsinkconfig-dumponfault) |
| `SdSinkConfig::enabled` | field | [sd_sink.md](sd_sink.md#sdsinkconfig-enabled) |
| `SdSinkConfig::flushOnFault` | field | [sd_sink.md](sd_sink.md#sdsinkconfig-flushonfault) |
| `SdSinkConfig::streamTicks` | field | [sd_sink.md](sd_sink.md#sdsinkconfig-streamticks) |
| `SdSinkStorage` | struct | [sd_sink.md](sd_sink.md#struct-sdsinkstorage) |
| `SdSinkStorage::buffer` | field | [sd_sink.md](sd_sink.md#sdsinkstorage-buffer) |
| `SdSinkStorage::ring` | field | [sd_sink.md](sd_sink.md#sdsinkstorage-ring) |
| `SessionInfo` | struct | [session_info.md](session_info.md#struct-sessioninfo) |
| `SessionInfo::alliance` | field | [session_info.md](session_info.md#sessioninfo-alliance) |
| `SessionInfo::buildHash` | field | [session_info.md](session_info.md#sessioninfo-buildhash) |
| `SessionInfo::portMap` | field | [session_info.md](session_info.md#sessioninfo-portmap) |
| `SessionInfo::routineId` | field | [session_info.md](session_info.md#sessioninfo-routineid) |
| `SessionInfo::side` | field | [session_info.md](session_info.md#sessioninfo-side) |
| `setPreconditionHandler` | free function | [check.md](check.md#setpreconditionhandler) |
| `SettleConfig` | struct | [settled_util.md](settled_util.md#struct-settleconfig) |
| `SettleConfig::maxError` | field | [settled_util.md](settled_util.md#settleconfig-maxerror) |
| `SettleConfig::maxErrorRate` | field | [settled_util.md](settled_util.md#settleconfig-maxerrorrate) |
| `SettleConfig::settleTime` | field | [settled_util.md](settled_util.md#settleconfig-settletime) |
| `SettledUtil` | class | [settled_util.md](settled_util.md#class-settledutil) |
| `SettledUtil::isSettled` | function | [settled_util.md](settled_util.md#settledutil-issettled) |
| `SettledUtil::reset` | function | [settled_util.md](settled_util.md#settledutil-reset) |
| `SettledUtil::SettledUtil` | function | [settled_util.md](settled_util.md#settledutil-settledutil) |
| `SettledUtil::update` | function | [settled_util.md](settled_util.md#settledutil-update) |
| `StallConfig` | struct | [stall_detector.md](stall_detector.md#struct-stallconfig) |
| `StallConfig::currentAtLeast` | field | [stall_detector.md](stall_detector.md#stallconfig-currentatleast) |
| `StallConfig::persistence` | field | [stall_detector.md](stall_detector.md#stallconfig-persistence) |
| `StallConfig::speedAtMost` | field | [stall_detector.md](stall_detector.md#stallconfig-speedatmost) |
| `StallDetector` | class | [stall_detector.md](stall_detector.md#class-stalldetector) |
| `StallDetector::config` | function | [stall_detector.md](stall_detector.md#stalldetector-config) |
| `StallDetector::reset` | function | [stall_detector.md](stall_detector.md#stalldetector-reset) |
| `StallDetector::StallDetector` | function | [stall_detector.md](stall_detector.md#stalldetector-stalldetector) |
| `StallDetector::update` | function | [stall_detector.md](stall_detector.md#stalldetector-update) |
| `StrafeTo` | class | [strafe_to.md](strafe_to.md#class-strafeto) |
| `StrafeTo::name` | function | [strafe_to.md](strafe_to.md#strafeto-name) |
| `StrafeTo::StrafeTo` | function | [strafe_to.md](strafe_to.md#strafeto-strafeto) |

## T

| Name | Kind | Page |
|---|---|---|
| `TagCorners` | struct | [vision_conversion.md](vision_conversion.md#struct-tagcorners) |
| `TagCorners::u` | field | [vision_conversion.md](vision_conversion.md#tagcorners-u) |
| `TagCorners::v` | field | [vision_conversion.md](vision_conversion.md#tagcorners-v) |
| `tagCornersToRobotPose` | free function | [vision_conversion.md](vision_conversion.md#tagcornerstorobotpose) |
| `TagMap` | class | [tag_map.md](tag_map.md#class-tagmap) |
| `TagMap::add` | function | [tag_map.md](tag_map.md#tagmap-add) |
| `TagMap::anyInvented` | function | [tag_map.md](tag_map.md#tagmap-anyinvented) |
| `TagMap::empty` | function | [tag_map.md](tag_map.md#tagmap-empty) |
| `TagMap::find` | function | [tag_map.md](tag_map.md#tagmap-find) |
| `TagMap::kMaxTags` | field | [tag_map.md](tag_map.md#tagmap-kmaxtags) |
| `TagMap::robotPoseFromTag` | function | [tag_map.md](tag_map.md#tagmap-robotposefromtag) |
| `TagMap::size` | function | [tag_map.md](tag_map.md#tagmap-size) |
| `TagObservation` | struct | [vision.md](vision.md#struct-tagobservation) |
| `TagObservation::confidence` | field | [vision.md](vision.md#tagobservation-confidence) |
| `TagObservation::id` | field | [vision.md](vision.md#tagobservation-id) |
| `TagObservation::poseInRobot` | field | [vision.md](vision.md#tagobservation-poseinrobot) |
| `TagPlacement` | struct | [tag_map.md](tag_map.md#struct-tagplacement) |
| `TagPlacement::fieldPose` | field | [tag_map.md](tag_map.md#tagplacement-fieldpose) |
| `TagPlacement::id` | field | [tag_map.md](tag_map.md#tagplacement-id) |
| `TagPlacement::provenance` | field | [tag_map.md](tag_map.md#tagplacement-provenance) |
| `TagPlacement::source` | field | [tag_map.md](tag_map.md#tagplacement-source) |
| `TagPnpResult` | struct | [vision_conversion.md](vision_conversion.md#struct-tagpnpresult) |
| `TagPnpResult::poseInRobot` | field | [vision_conversion.md](vision_conversion.md#tagpnpresult-poseinrobot) |
| `TagPnpResult::range` | field | [vision_conversion.md](vision_conversion.md#tagpnpresult-range) |
| `TagPnpResult::reprojectionError` | field | [vision_conversion.md](vision_conversion.md#tagpnpresult-reprojectionerror) |
| `TagPnpResult::valid` | field | [vision_conversion.md](vision_conversion.md#tagpnpresult-valid) |
| `TagProvenance` | enum class | [tag_map.md](tag_map.md#enum-class-tagprovenance) |
| `TagProvenance::Invented` | enumerator | [tag_map.md](tag_map.md#tagprovenance-invented) |
| `TagProvenance::Measured` | enumerator | [tag_map.md](tag_map.md#tagprovenance-measured) |
| `TagProvenance::Specified` | enumerator | [tag_map.md](tag_map.md#tagprovenance-specified) |
| `TagProvenance::Unspecified` | enumerator | [tag_map.md](tag_map.md#tagprovenance-unspecified) |
| `TankKinematics` | class | [tank.md](tank.md#class-tankkinematics) |
| `TankKinematics::desaturate` | function | [tank.md](tank.md#tankkinematics-desaturate) |
| `TankKinematics::forward` | function | [tank.md](tank.md#tankkinematics-forward) |
| `TankKinematics::strafeAuthority` | function | [tank.md](tank.md#tankkinematics-strafeauthority) |
| `TankKinematics::TankKinematics` | function | [tank.md](tank.md#tankkinematics-tankkinematics) |
| `TankKinematics::toWheels` | function | [tank.md](tank.md#tankkinematics-towheels) |
| `TankKinematics::wheelCount` | function | [tank.md](tank.md#tankkinematics-wheelcount) |
| `TermSink` | class | [term_sink.md](term_sink.md#class-termsink) |
| `TermSink::emit` | function | [term_sink.md](term_sink.md#termsink-emit) |
| `TermSink::log` | function | [term_sink.md](term_sink.md#termsink-log) |
| `TermSink::summarize` | function | [term_sink.md](term_sink.md#termsink-summarize) |
| `TermSink::TermSink` | function | [term_sink.md](term_sink.md#termsink-termsink) |
| `TermSink::wantsRecord` | function | [term_sink.md](term_sink.md#termsink-wantsrecord) |
| `throwingPreconditionHandler` | free function | [check.md](check.md#throwingpreconditionhandler) |
| `TickAttribution` | class | [tick_attribution.md](tick_attribution.md#class-tickattribution) |
| `TickAttribution::abandonTick` | function | [tick_attribution.md](tick_attribution.md#tickattribution-abandontick) |
| `TickAttribution::beginTick` | function | [tick_attribution.md](tick_attribution.md#tickattribution-begintick) |
| `TickAttribution::endTick` | function | [tick_attribution.md](tick_attribution.md#tickattribution-endtick) |
| `TickAttribution::hasCompletedTick` | function | [tick_attribution.md](tick_attribution.md#tickattribution-hascompletedtick) |
| `TickAttribution::lastAttributed` | function | [tick_attribution.md](tick_attribution.md#tickattribution-lastattributed) |
| `TickAttribution::lastOther` | function | [tick_attribution.md](tick_attribution.md#tickattribution-lastother) |
| `TickAttribution::lastPhases` | function | [tick_attribution.md](tick_attribution.md#tickattribution-lastphases) |
| `TickAttribution::lastTotal` | function | [tick_attribution.md](tick_attribution.md#tickattribution-lasttotal) |
| `TickAttribution::lastWorstPhase` | function | [tick_attribution.md](tick_attribution.md#tickattribution-lastworstphase) |
| `TickAttribution::phase` | function | [tick_attribution.md](tick_attribution.md#tickattribution-phase) |
| `TickAttribution::Phases` | alias | [tick_attribution.md](tick_attribution.md#tickattribution-phases) |
| `TickAttribution::PhaseScope` | class | [tick_attribution.md](tick_attribution.md#class-tickattribution-phasescope) |
| `TickAttribution::PhaseScope::operator=` | function | [tick_attribution.md](tick_attribution.md#tickattribution-phasescope-operator-eq) |
| `TickAttribution::PhaseScope::PhaseScope` | function | [tick_attribution.md](tick_attribution.md#tickattribution-phasescope-phasescope) |
| `TickAttribution::PhaseScope::PhaseScope (overload 2)` | function | [tick_attribution.md](tick_attribution.md#tickattribution-phasescope-phasescope-2) |
| `TickAttribution::PhaseScope::~PhaseScope` | function | [tick_attribution.md](tick_attribution.md#tickattribution-phasescope-destructor-phasescope) |
| `TickAttribution::reset` | function | [tick_attribution.md](tick_attribution.md#tickattribution-reset) |
| `TickAttribution::TickAttribution` | function | [tick_attribution.md](tick_attribution.md#tickattribution-tickattribution) |
| `tickHealthObservables` | free function | [motion.md](motion.md#tickhealthobservables) |
| `TickPhase` | enum class | [debug_record.md](debug_record.md#enum-class-tickphase) |
| `TickPhase::Health` | enumerator | [debug_record.md](debug_record.md#tickphase-health) |
| `TickPhase::Localization` | enumerator | [debug_record.md](debug_record.md#tickphase-localization) |
| `TickPhase::Motion` | enumerator | [debug_record.md](debug_record.md#tickphase-motion) |
| `TickPhase::Scheduler` | enumerator | [debug_record.md](debug_record.md#tickphase-scheduler) |
| `TickPhase::Telemetry` | enumerator | [debug_record.md](debug_record.md#tickphase-telemetry) |
| `TickPhase::User` | enumerator | [debug_record.md](debug_record.md#tickphase-user) |
| `tickPhaseName` | free function | [tick_attribution.md](tick_attribution.md#tickphasename) |
| `Time` | type alias | [quantity.md](quantity.md#time) |
| `TrackingWheel` | class | [tracking_wheel.md](tracking_wheel.md#class-trackingwheel) |
| `TrackingWheel::forward` | function | [tracking_wheel.md](tracking_wheel.md#trackingwheel-forward) |
| `TrackingWheel::lateral` | function | [tracking_wheel.md](tracking_wheel.md#trackingwheel-lateral) |
| `TrackingWheel::offset` | function | [tracking_wheel.md](tracking_wheel.md#trackingwheel-offset) |
| `TrackingWheel::reset` | function | [tracking_wheel.md](tracking_wheel.md#trackingwheel-reset) |
| `TrackingWheel::Role` | enum class | [tracking_wheel.md](tracking_wheel.md#enum-class-trackingwheel-role) |
| `TrackingWheel::role` | function | [tracking_wheel.md](tracking_wheel.md#trackingwheel-role) |
| `TrackingWheel::Role::Forward` | enumerator | [tracking_wheel.md](tracking_wheel.md#trackingwheel-role-forward) |
| `TrackingWheel::Role::Lateral` | enumerator | [tracking_wheel.md](tracking_wheel.md#trackingwheel-role-lateral) |
| `TrackingWheel::travelDelta` | function | [tracking_wheel.md](tracking_wheel.md#trackingwheel-traveldelta) |
| `TrajectoryResult` | struct | [chassis.md](chassis.md#struct-trajectoryresult) |
| `TrajectoryResult::completedLegs` | field | [chassis.md](chassis.md#trajectoryresult-completedlegs) |
| `TrajectoryResult::exit` | field | [chassis.md](chassis.md#trajectoryresult-exit) |
| `TrajectoryResult::succeeded` | function | [chassis.md](chassis.md#trajectoryresult-succeeded) |
| `TrajectoryResult::totalLegs` | field | [chassis.md](chassis.md#trajectoryresult-totallegs) |
| `TrapezoidProfile` | class | [trapezoid_profile.md](trapezoid_profile.md#class-trapezoidprofile) |
| `TrapezoidProfile::duration` | function | [trapezoid_profile.md](trapezoid_profile.md#trapezoidprofile-duration) |
| `TrapezoidProfile::isDone` | function | [trapezoid_profile.md](trapezoid_profile.md#trapezoidprofile-isdone) |
| `TrapezoidProfile::sample` | function | [trapezoid_profile.md](trapezoid_profile.md#trapezoidprofile-sample) |
| `TrapezoidProfile::TrapezoidProfile` | function | [trapezoid_profile.md](trapezoid_profile.md#trapezoidprofile-trapezoidprofile) |
| `TriageInfo` | struct | [blackbox_format.md](blackbox_format.md#struct-triageinfo) |
| `TriageInfo::brownout` | field | [blackbox_format.md](blackbox_format.md#triageinfo-brownout) |
| `TriageInfo::fault` | field | [blackbox_format.md](blackbox_format.md#triageinfo-fault) |
| `TriageInfo::faultTime` | field | [blackbox_format.md](blackbox_format.md#triageinfo-faulttime) |
| `TriageInfo::precedingTicks` | field | [blackbox_format.md](blackbox_format.md#triageinfo-precedingticks) |
| `TriageInfo::tickIndex` | field | [blackbox_format.md](blackbox_format.md#triageinfo-tickindex) |
| `TurnTo` | class | [turn_to.md](turn_to.md#class-turnto) |
| `TurnTo::cancel` | function | [turn_to.md](turn_to.md#turnto-cancel) |
| `TurnTo::exitReason` | function | [turn_to.md](turn_to.md#turnto-exitreason) |
| `TurnTo::name` | function | [turn_to.md](turn_to.md#turnto-name) |
| `TurnTo::start` | function | [turn_to.md](turn_to.md#turnto-start) |
| `TurnTo::state` | function | [turn_to.md](turn_to.md#turnto-state) |
| `TurnTo::target` | function | [turn_to.md](turn_to.md#turnto-target) |
| `TurnTo::tick` | function | [turn_to.md](turn_to.md#turnto-tick) |
| `TurnTo::TurnTo` | function | [turn_to.md](turn_to.md#turnto-turnto) |
| `Twist2d` | class | [twist2d.md](twist2d.md#class-twist2d) |
| `Twist2d::approxEqual` | function | [twist2d.md](twist2d.md#twist2d-approxequal) |
| `Twist2d::omega` | function | [twist2d.md](twist2d.md#twist2d-omega) |
| `Twist2d::Twist2d` | function | [twist2d.md](twist2d.md#twist2d-twist2d) |
| `Twist2d::Twist2d (overload 2)` | function | [twist2d.md](twist2d.md#twist2d-twist2d-2) |
| `Twist2d::vx` | function | [twist2d.md](twist2d.md#twist2d-vx) |
| `Twist2d::vy` | function | [twist2d.md](twist2d.md#twist2d-vy) |

## V

| Name | Kind | Page |
|---|---|---|
| `Velocity` | type alias | [quantity.md](quantity.md#velocity) |
| `Voltage` | type alias | [quantity.md](quantity.md#voltage) |

## W

| Name | Kind | Page |
|---|---|---|
| `WaitResult` | enum class | [motion_scheduler.md](motion_scheduler.md#enum-class-waitresult) |
| `WaitResult::Satisfied` | enumerator | [motion_scheduler.md](motion_scheduler.md#waitresult-satisfied) |
| `WaitResult::TimedOut` | enumerator | [motion_scheduler.md](motion_scheduler.md#waitresult-timedout) |
| `Watchdog` | class | [watchdog.md](watchdog.md#class-watchdog) |
| `Watchdog::elapsed` | function | [watchdog.md](watchdog.md#watchdog-elapsed) |
| `Watchdog::expired` | function | [watchdog.md](watchdog.md#watchdog-expired) |
| `Watchdog::reset` | function | [watchdog.md](watchdog.md#watchdog-reset) |
| `Watchdog::start` | function | [watchdog.md](watchdog.md#watchdog-start) |
| `Watchdog::started` | function | [watchdog.md](watchdog.md#watchdog-started) |
| `Watchdog::Watchdog` | function | [watchdog.md](watchdog.md#watchdog-watchdog) |
| `WheelSpeeds` | class | [wheel_speeds.md](wheel_speeds.md#class-wheelspeeds) |
| `WheelSpeeds::approxEqual` | function | [wheel_speeds.md](wheel_speeds.md#wheelspeeds-approxequal) |
| `WheelSpeeds::kMaxWheels` | field | [wheel_speeds.md](wheel_speeds.md#wheelspeeds-kmaxwheels) |
| `WheelSpeeds::maxMagnitude` | function | [wheel_speeds.md](wheel_speeds.md#wheelspeeds-maxmagnitude) |
| `WheelSpeeds::operator[]` | function | [wheel_speeds.md](wheel_speeds.md#wheelspeeds-operator-index) |
| `WheelSpeeds::set` | function | [wheel_speeds.md](wheel_speeds.md#wheelspeeds-set) |
| `WheelSpeeds::size` | function | [wheel_speeds.md](wheel_speeds.md#wheelspeeds-size) |
| `WheelSpeeds::WheelSpeeds` | function | [wheel_speeds.md](wheel_speeds.md#wheelspeeds-wheelspeeds) |
| `WheelSpeeds::WheelSpeeds (overload 2)` | function | [wheel_speeds.md](wheel_speeds.md#wheelspeeds-wheelspeeds-2) |

## X

| Name | Kind | Page |
|---|---|---|
| `xDrive` | free function | [x_drive.md](x_drive.md#xdrive) |

## Y

| Name | Kind | Page |
|---|---|---|
| `YawRateSource` | enum class | [pros-imu.md](pros-imu.md#enum-class-yawratesource) |
| `YawRateSource::DifferentiateRotation` | enumerator | [pros-imu.md](pros-imu.md#yawratesource-differentiaterotation) |
| `YawRateSource::GyroRateZ` | enumerator | [pros-imu.md](pros-imu.md#yawratesource-gyroratez) |
