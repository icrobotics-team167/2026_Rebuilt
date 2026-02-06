// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import static java.util.Map.entry;

import edu.wpi.first.math.geometry.Rotation2d;

public final class HubShotMap extends ShotMap {
  public HubShotMap() {
    put(
        0.5,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.396374, new Rotation2d(0.000000), 5.253808)),
                    entry(1.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 9.261271)),
                    entry(2.000000, new ShotResult(1.360690, new Rotation2d(3.141593), 5.294359)),
                    entry(3.000000, new ShotResult(1.184663, new Rotation2d(3.141593), 5.589707)),
                    entry(4.000000, new ShotResult(1.030443, new Rotation2d(-3.141593), 6.038884)),
                    entry(5.000000, new ShotResult(0.900129, new Rotation2d(-3.141593), 6.610869)),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.396374, new Rotation2d(0.000000), 5.253808)),
                    entry(1.000000, new ShotResult(1.428441, new Rotation2d(-1.299441), 5.228377)),
                    entry(2.000000, new ShotResult(1.286656, new Rotation2d(-1.917152), 5.392916)),
                    entry(3.000000, new ShotResult(1.127958, new Rotation2d(-2.091833), 5.729825)),
                    entry(4.000000, new ShotResult(0.985405, new Rotation2d(-2.168285), 6.211774)),
                    entry(5.000000, new ShotResult(0.864116, new Rotation2d(-2.210683), 6.808348)),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.396374, new Rotation2d(0.000000), 5.253808)),
                    entry(1.000000, new ShotResult(1.314492, new Rotation2d(-0.835858), 5.349419)),
                    entry(2.000000, new ShotResult(1.168210, new Rotation2d(-1.146346), 5.624794)),
                    entry(3.000000, new ShotResult(1.024922, new Rotation2d(-1.278166), 6.055248)),
                    entry(4.000000, new ShotResult(0.899236, new Rotation2d(-1.348569), 6.610860)),
                    entry(5.000000, new ShotResult(0.793111, new Rotation2d(-1.391960), 7.263224)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.477876), 8.571639)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.396374, new Rotation2d(0.000000), 5.253808)),
                    entry(1.000000, new ShotResult(1.241557, new Rotation2d(-0.413613), 5.467789)),
                    entry(2.000000, new ShotResult(1.086010, new Rotation2d(-0.547799), 5.847793)),
                    entry(3.000000, new ShotResult(0.949154, new Rotation2d(-0.611544), 6.364321)),
                    entry(4.000000, new ShotResult(0.833627, new Rotation2d(-0.648490), 6.987410)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.697751), 7.813345)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.728978), 9.073166)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.396374, new Rotation2d(0.000000), 5.253808)),
                    entry(1.000000, new ShotResult(1.216720, new Rotation2d(-0.000000), 5.516073)),
                    entry(2.000000, new ShotResult(1.057847, new Rotation2d(-0.000000), 5.937741)),
                    entry(3.000000, new ShotResult(0.922846, new Rotation2d(-0.000000), 6.488072)),
                    entry(4.000000, new ShotResult(0.810567, new Rotation2d(-0.000000), 7.137608)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.011539)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 9.254184))))));
    put(
        1.0751434584179567,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(1.418486, new Rotation2d(0.000000), 5.294738)),
                    entry(2.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 5.473441)),
                    entry(3.000000, new ShotResult(1.343174, new Rotation2d(3.141593), 5.380921)),
                    entry(4.000000, new ShotResult(1.170995, new Rotation2d(3.141593), 5.691487)),
                    entry(5.000000, new ShotResult(1.020490, new Rotation2d(3.141593), 6.151682)),
                    entry(6.000000, new ShotResult(0.893314, new Rotation2d(3.141593), 6.731164)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(1.326203, new Rotation2d(-0.576664), 5.394560)),
                    entry(2.000000, new ShotResult(1.296397, new Rotation2d(-1.308074), 5.440063)),
                    entry(3.000000, new ShotResult(1.180232, new Rotation2d(-1.723540), 5.665065)),
                    entry(4.000000, new ShotResult(1.047462, new Rotation2d(-1.921135), 6.049271)),
                    entry(5.000000, new ShotResult(0.924332, new Rotation2d(-2.028265), 6.565374)),
                    entry(
                        6.000000, new ShotResult(0.817566, new Rotation2d(-2.094050), 7.185398)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(1.195082, new Rotation2d(-0.508465), 5.625933)),
                    entry(2.000000, new ShotResult(1.094831, new Rotation2d(-0.839559), 5.888706)),
                    entry(3.000000, new ShotResult(0.980306, new Rotation2d(-1.031765), 6.301676)),
                    entry(4.000000, new ShotResult(0.871911, new Rotation2d(-1.149115), 6.837852)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-1.244514), 7.491661)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.377611), 8.700985)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(1.108061, new Rotation2d(-0.275513), 5.847669)),
                    entry(2.000000, new ShotResult(0.978707, new Rotation2d(-0.415176), 6.305510)),
                    entry(3.000000, new ShotResult(0.863890, new Rotation2d(-0.496527), 6.879975)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.562759), 7.524733)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.637700), 8.446133)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.680262), 9.628133)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, new ShotResult(0.941626, new Rotation2d(-0.000000), 6.470283)),
                    entry(3.000000, new ShotResult(0.827181, new Rotation2d(-0.000000), 7.105738)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 7.784537)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.763378)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 9.942739))))));
    put(
        1.650286916835913,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.117411, new Rotation2d(0.000000), 5.910645)),
                    entry(1.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 14.165487)),
                    entry(2.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 7.352065)),
                    entry(3.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 5.368273)),
                    entry(4.000000, new ShotResult(1.308338, new Rotation2d(-3.141593), 5.518598)),
                    entry(5.000000, new ShotResult(1.142603, new Rotation2d(3.141593), 5.859588)),
                    entry(6.000000, new ShotResult(0.998503, new Rotation2d(3.141593), 6.342652)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.117411, new Rotation2d(0.000000), 5.910645)),
                    entry(1.000000, new ShotResult(1.209159, new Rotation2d(-0.361694), 5.682189)),
                    entry(2.000000, new ShotResult(1.237216, new Rotation2d(-0.882793), 5.627345)),
                    entry(3.000000, new ShotResult(1.181560, new Rotation2d(-1.359004), 5.750530)),
                    entry(4.000000, new ShotResult(1.078747, new Rotation2d(-1.659094), 6.039359)),
                    entry(5.000000, new ShotResult(0.966252, new Rotation2d(-1.835155), 6.471854)),
                    entry(
                        6.000000, new ShotResult(0.861074, new Rotation2d(-1.944516), 7.022049)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.117411, new Rotation2d(0.000000), 5.910645)),
                    entry(1.000000, new ShotResult(1.089281, new Rotation2d(-0.370305), 5.995573)),
                    entry(2.000000, new ShotResult(1.018396, new Rotation2d(-0.660172), 6.243052)),
                    entry(3.000000, new ShotResult(0.929154, new Rotation2d(-0.861258), 6.634540)),
                    entry(4.000000, new ShotResult(0.838681, new Rotation2d(-0.998608), 7.146459)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-1.143562), 7.804665)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.288517), 8.898428)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.117411, new Rotation2d(0.000000), 5.910645)),
                    entry(1.000000, new ShotResult(1.005121, new Rotation2d(-0.212162), 6.292808)),
                    entry(2.000000, new ShotResult(0.895934, new Rotation2d(-0.340662), 6.802499)),
                    entry(3.000000, new ShotResult(0.798368, new Rotation2d(-0.424270), 7.413578)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.521805), 8.065767)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.593768), 9.003958)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.641810), 10.139307)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.117411, new Rotation2d(0.000000), 5.910645)),
                    entry(1.000000, new ShotResult(0.976360, new Rotation2d(-0.000000), 6.411820)),
                    entry(2.000000, new ShotResult(0.857617, new Rotation2d(-0.000000), 7.021079)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 7.660340)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.413366)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.403146)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 10.551533))))));
    put(
        2.2254303752538696,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.023108, new Rotation2d(0.000000), 6.331734)),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 10.303857)),
                    entry(3.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 6.147453)),
                    entry(4.000000, new ShotResult(1.438168, new Rotation2d(3.141593), 5.478421)),
                    entry(5.000000, new ShotResult(1.263225, new Rotation2d(3.141593), 5.698601)),
                    entry(6.000000, new ShotResult(1.105645, new Rotation2d(3.141593), 6.078579)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.023108, new Rotation2d(0.000000), 6.331734)),
                    entry(1.000000, new ShotResult(1.110010, new Rotation2d(-0.268175), 6.036975)),
                    entry(2.000000, new ShotResult(1.159878, new Rotation2d(-0.648261), 5.901185)),
                    entry(3.000000, new ShotResult(1.148336, new Rotation2d(-1.069922), 5.935590)),
                    entry(4.000000, new ShotResult(1.082020, new Rotation2d(-1.411102), 6.136082)),
                    entry(5.000000, new ShotResult(0.989365, new Rotation2d(-1.641506), 6.486654)),
                    entry(
                        6.000000, new ShotResult(0.892538, new Rotation2d(-1.792549), 6.965056)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.023108, new Rotation2d(0.000000), 6.331734)),
                    entry(1.000000, new ShotResult(1.003208, new Rotation2d(-0.295943), 6.411132)),
                    entry(2.000000, new ShotResult(0.950669, new Rotation2d(-0.547561), 6.643411)),
                    entry(3.000000, new ShotResult(0.880367, new Rotation2d(-0.740814), 7.013114)),
                    entry(4.000000, new ShotResult(0.805179, new Rotation2d(-0.883939), 7.499948)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-1.062678), 8.139321)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.211202), 9.140186)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.023108, new Rotation2d(0.000000), 6.331734)),
                    entry(1.000000, new ShotResult(0.925485, new Rotation2d(-0.175563), 6.764001)),
                    entry(2.000000, new ShotResult(0.831619, new Rotation2d(-0.292718), 7.309233)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.392885), 7.875981)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.488289), 8.584005)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.559127), 9.516221)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.610144), 10.620153)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(1.023108, new Rotation2d(0.000000), 6.331734)),
                    entry(1.000000, new ShotResult(0.898655, new Rotation2d(-0.000000), 6.904779)),
                    entry(2.000000, new ShotResult(0.794485, new Rotation2d(-0.000000), 7.567647)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.172835)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.987354)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.978412)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 11.110044))))));
    put(
        2.800573833671826,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.948587, new Rotation2d(0.000000), 6.773037)),
                    entry(1.000000, new ShotResult(1.079620, new Rotation2d(0.000000), 6.245265)),
                    entry(2.000000, new ShotResult(1.231521, new Rotation2d(0.000000), 5.842260)),
                    entry(3.000000, new ShotResult(1.401218, new Rotation2d(0.000000), 5.590772)),
                    entry(4.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 5.824298)),
                    entry(5.000000, new ShotResult(1.379361, new Rotation2d(-3.141593), 5.643323)),
                    entry(
                        6.000000, new ShotResult(1.212585, new Rotation2d(-3.141593), 5.917322)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.948587, new Rotation2d(0.000000), 6.773037)),
                    entry(1.000000, new ShotResult(1.028727, new Rotation2d(-0.216231), 6.427772)),
                    entry(2.000000, new ShotResult(1.085840, new Rotation2d(-0.512528), 6.227431)),
                    entry(3.000000, new ShotResult(1.099991, new Rotation2d(-0.864914), 6.186666)),
                    entry(4.000000, new ShotResult(1.065100, new Rotation2d(-1.198853), 6.307956)),
                    entry(5.000000, new ShotResult(0.995771, new Rotation2d(-1.458590), 6.581366)),
                    entry(
                        6.000000, new ShotResult(0.912101, new Rotation2d(-1.643002), 6.989026)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.948587, new Rotation2d(0.000000), 6.773037)),
                    entry(1.000000, new ShotResult(0.933653, new Rotation2d(-0.249469), 6.847392)),
                    entry(2.000000, new ShotResult(0.893125, new Rotation2d(-0.471238), 7.065618)),
                    entry(3.000000, new ShotResult(0.836639, new Rotation2d(-0.652561), 7.414815)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.808593), 7.864531)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.996283), 8.481663)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.144427), 9.409161)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.948587, new Rotation2d(0.000000), 6.773037)),
                    entry(1.000000, new ShotResult(0.862675, new Rotation2d(-0.151528), 7.242006)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.260742), 7.800647)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.370826), 8.332963)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.460814), 9.072275)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.530652), 9.996558)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.583339), 11.078462)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.948587, new Rotation2d(0.000000), 6.773037)),
                    entry(1.000000, new ShotResult(0.837944, new Rotation2d(-0.000000), 7.399150)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.009407)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.675251)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.518187)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.510332)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 11.633557))))));
    put(
        3.375717292089783,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.888532, new Rotation2d(0.000000), 7.220785)),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 13.557857)),
                    entry(3.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 8.898562)),
                    entry(4.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 6.528902)),
                    entry(5.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 5.691272)),
                    entry(
                        6.000000, new ShotResult(1.317215, new Rotation2d(-3.141593), 5.842834)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.888532, new Rotation2d(0.000000), 7.220785)),
                    entry(1.000000, new ShotResult(0.961951, new Rotation2d(-0.183082), 6.836539)),
                    entry(2.000000, new ShotResult(1.020327, new Rotation2d(-0.426290), 6.584615)),
                    entry(3.000000, new ShotResult(1.048398, new Rotation2d(-0.721852), 6.481072)),
                    entry(4.000000, new ShotResult(1.036492, new Rotation2d(-1.028124), 6.532836)),
                    entry(5.000000, new ShotResult(0.989210, new Rotation2d(-1.294653), 6.735432)),
                    entry(
                        6.000000, new ShotResult(0.920956, new Rotation2d(-1.500714), 7.075478)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.888532, new Rotation2d(0.000000), 7.220785)),
                    entry(1.000000, new ShotResult(0.876850, new Rotation2d(-0.217541), 7.290655)),
                    entry(2.000000, new ShotResult(0.844565, new Rotation2d(-0.416232), 7.496260)),
                    entry(3.000000, new ShotResult(0.798248, new Rotation2d(-0.585511), 7.826758)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.765311), 8.221794)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.940579), 8.825471)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.086524), 9.694466)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.888532, new Rotation2d(0.000000), 7.220785)),
                    entry(1.000000, new ShotResult(0.812037, new Rotation2d(-0.134417), 7.717196)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.248092), 8.200505)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.351663), 8.781155)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.437793), 9.535330)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.506597), 10.452893)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.560204), 11.519349)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.888532, new Rotation2d(0.000000), 7.220785)),
                    entry(1.000000, new ShotResult(0.789263, new Rotation2d(-0.000000), 7.886914)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.443890)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.155646)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.016191)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 11.010780)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 12.131390))))));
    put(
        3.950860750507739,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.839191, new Rotation2d(0.000000), 7.667423)),
                    entry(1.000000, new ShotResult(0.945816, new Rotation2d(0.000000), 7.043468)),
                    entry(2.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 14.467576)),
                    entry(3.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 10.546727)),
                    entry(4.000000, new ShotResult(1.377764, new Rotation2d(0.000000), 5.831825)),
                    entry(5.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 6.182406)),
                    entry(
                        6.000000, new ShotResult(1.417708, new Rotation2d(-3.141593), 5.842101)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.839191, new Rotation2d(0.000000), 7.667423)),
                    entry(1.000000, new ShotResult(0.906495, new Rotation2d(-0.159994), 7.252670)),
                    entry(2.000000, new ShotResult(0.963603, new Rotation2d(-0.367087), 6.959237)),
                    entry(3.000000, new ShotResult(0.998883, new Rotation2d(-0.619524), 6.803444)),
                    entry(4.000000, new ShotResult(1.002584, new Rotation2d(-0.894167), 6.794941)),
                    entry(5.000000, new ShotResult(0.973747, new Rotation2d(-1.153139), 6.933647)),
                    entry(
                        6.000000, new ShotResult(0.920953, new Rotation2d(-1.369388), 7.210430)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.839191, new Rotation2d(0.000000), 7.667423)),
                    entry(1.000000, new ShotResult(0.829761, new Rotation2d(-0.194153), 7.733339)),
                    entry(2.000000, new ShotResult(0.803364, new Rotation2d(-0.374676), 7.927732)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.548324), 8.199765)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.727074), 8.581270)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.893016), 9.167723)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.035956), 9.989385)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.839191, new Rotation2d(0.000000), 7.667423)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.123983), 8.145614)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.236545), 8.605984)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.335088), 9.214943)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.418132), 9.977723)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.485877), 10.890426)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.539945), 11.946462)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.839191, new Rotation2d(0.000000), 7.667423)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.272129)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.873121)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.614818)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.488602)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 11.487321)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 12.609844))))));
    put(
        4.526004208925696,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.797913, new Rotation2d(0.000000), 8.108749)),
                    entry(1.000000, new ShotResult(0.895027, new Rotation2d(0.000000), 7.451717)),
                    entry(2.000000, new ShotResult(1.009442, new Rotation2d(0.000000), 6.876576)),
                    entry(3.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 11.978964)),
                    entry(4.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 8.445873)),
                    entry(5.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 6.787949)),
                    entry(
                        6.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 6.006263)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.797913, new Rotation2d(0.000000), 8.108749)),
                    entry(1.000000, new ShotResult(0.859826, new Rotation2d(-0.142924), 7.669829)),
                    entry(2.000000, new ShotResult(0.914629, new Rotation2d(-0.323993), 7.342707)),
                    entry(3.000000, new ShotResult(0.953354, new Rotation2d(-0.543772), 7.143355)),
                    entry(4.000000, new ShotResult(0.967288, new Rotation2d(-0.789185), 7.082934)),
                    entry(5.000000, new ShotResult(0.952857, new Rotation2d(-1.033600), 7.164637)),
                    entry(
                        6.000000, new ShotResult(0.914154, new Rotation2d(-1.251067), 7.383096)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.797913, new Rotation2d(0.000000), 8.108749)),
                    entry(1.000000, new ShotResult(0.790112, new Rotation2d(-0.176216), 8.171182)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.350178), 8.313279)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.524116), 8.553036)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.693456), 8.937580)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.851825), 9.506921)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.991445), 10.289772)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.797913, new Rotation2d(0.000000), 8.108749)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.118771), 8.513684)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.226237), 9.006436)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.320654), 9.634255)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.401082), 10.403057)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.467767), 11.312846)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.522005), 12.362577)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.797913, new Rotation2d(0.000000), 8.108749)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.653272)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.291646)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.055401)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.940588)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 11.945207)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 13.073534))))));
    put(
        5.101147667343652,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.480999)),
                    entry(1.000000, new ShotResult(0.851842, new Rotation2d(0.000000), 7.859129)),
                    entry(2.000000, new ShotResult(0.956660, new Rotation2d(0.000000), 7.247883)),
                    entry(3.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 13.116116)),
                    entry(4.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 9.591222)),
                    entry(5.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 7.491537)),
                    entry(
                        6.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 6.465536)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.480999)),
                    entry(1.000000, new ShotResult(0.820020, new Rotation2d(-0.129749), 8.084164)),
                    entry(2.000000, new ShotResult(0.872160, new Rotation2d(-0.291207), 7.729483)),
                    entry(3.000000, new ShotResult(0.912209, new Rotation2d(-0.485814), 7.493676)),
                    entry(4.000000, new ShotResult(0.932709, new Rotation2d(-0.706046), 7.388622)),
                    entry(5.000000, new ShotResult(0.929104, new Rotation2d(-0.933536), 7.419819)),
                    entry(
                        6.000000, new ShotResult(0.902487, new Rotation2d(-1.146251), 7.585055)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.480999)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.168274), 8.524320)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.336114), 8.660088)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.502256), 8.905672)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.663787), 9.288817)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.815736), 9.842357)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.951971), 10.593041)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.480999)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.114004), 8.882562)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.217066), 9.398583)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.307971), 10.040301)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.386113), 10.814154)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.451756), 11.722933)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.505977), 12.769916)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.480999)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.032026)),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.480031)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 11.375991)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 12.388325)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 13.526071))))));
    put(
        5.676291125761609,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.822621)),
                    entry(1.000000, new ShotResult(0.814637, new Rotation2d(0.000000), 8.262866)),
                    entry(2.000000, new ShotResult(0.911129, new Rotation2d(0.000000), 7.622410)),
                    entry(3.000000, new ShotResult(1.023913, new Rotation2d(0.000000), 7.063214)),
                    entry(4.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 10.767553)),
                    entry(5.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 8.282921)),
                    entry(
                        6.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 6.995463)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.822621)),
                    entry(1.000000, new ShotResult(0.785627, new Rotation2d(-0.119245), 8.493304)),
                    entry(2.000000, new ShotResult(0.835064, new Rotation2d(-0.265400), 8.115924)),
                    entry(3.000000, new ShotResult(0.875233, new Rotation2d(-0.440181), 7.849476)),
                    entry(4.000000, new ShotResult(0.899854, new Rotation2d(-0.639214), 7.706043)),
                    entry(5.000000, new ShotResult(0.904208, new Rotation2d(-0.849831), 7.692673)),
                    entry(
                        6.000000, new ShotResult(0.887563, new Rotation2d(-1.054350), 7.809680)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.822621)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.161968), 8.867397)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.323264), 9.006570)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.482594), 9.254563)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.637448), 9.634363)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.783812), 10.173752)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.916722), 10.897569)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.822621)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.109684), 9.247708)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.208884), 9.781654)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.296734), 10.434527)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.372839), 11.213262)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.437473), 12.122881)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.491556), 13.170354)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.822621)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.405086)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.094091)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.891015)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 11.797782)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 12.819716)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 13.970468))))));
  }
}
