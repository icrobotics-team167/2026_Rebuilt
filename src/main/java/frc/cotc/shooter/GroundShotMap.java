// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import static java.util.Map.entry;

import edu.wpi.first.math.geometry.Rotation2d;

public final class GroundShotMap extends ShotMap {
  public GroundShotMap() {
    put(
        0.5,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 1.565821)),
                    entry(1.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 2.301772)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-3.141593), 1.098253)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(3.141593), 3.078279)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-3.141593), 4.776304)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-3.141593), 6.365421)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-3.141593), 7.909116)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 1.565821)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.968435), 1.213551)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-1.858927), 2.085917)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-2.142870), 3.568347)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-2.239057), 5.098765)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-2.281755), 6.609296)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-2.304381), 8.108830)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 1.565821)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.780265), 2.010272)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-1.179147), 3.060162)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-1.352499), 4.346765)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-1.434688), 5.714547)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-1.478411), 7.117155)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.504022), 8.544891)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 1.565821)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.410766), 2.504245)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.579890), 3.650374)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.659836), 4.896104)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.702142), 6.200966)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.726598), 7.547848)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.741778), 8.932009)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 1.565821)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 2.669811)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 3.848032)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 5.088429)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 6.378514)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 7.710103)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 9.081254))))));
    put(
        2.489352127470445,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 10.684966)),
                    entry(3.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 4.351321)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 1.834311)),
                    entry(5.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 1.610443)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 0.812848)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.557319), 3.782207)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.928916), 3.746269)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-1.339959), 4.110338)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-1.713359), 5.054950)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.958662), 6.493452)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.304926), 4.713211)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.587815), 5.104342)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.829742), 5.757570)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-1.020024), 6.652044)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-1.159737), 7.741325)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.258511), 8.972873)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.191645), 5.254532)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.336094), 6.073029)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.442776), 7.019403)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.520679), 8.074672)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.577487), 9.223433)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.619206), 10.456830)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 5.458000)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 6.411189)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 7.440773)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.543260)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.716585)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 10.962644))))));
    put(
        4.47870425494089,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 5.718912)),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 11.115227)),
                    entry(4.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 6.620521)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 3.295690)),
                    entry(6.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 3.476159)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.168350), 5.979648)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.365137), 5.609829)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.591826), 5.384942)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.848984), 5.336138)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-1.133765), 5.526946)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.431639), 6.070752)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.624088), 7.286678)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.799007), 7.931185)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.947944), 8.764534)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.069321), 9.772278)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.141753), 7.101838)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.259139), 7.841045)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.354825), 8.691642)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.431823), 9.647942)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.493262), 10.705676)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.542129), 11.864834)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.183933)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.231731)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 11.358256)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 12.573147))))));
    put(
        6.468056382411334,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.011208)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 7.257420)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 6.564538)),
                    entry(3.000000, ShotResult.invalid),
                    entry(4.000000, new ShotResult(1.483530, new Rotation2d(-3.141593), 11.461933)),
                    entry(5.000000, new ShotResult(1.483530, new Rotation2d(3.141593), 8.005761)),
                    entry(6.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 4.324354)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.011208)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.134273), 7.507606)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.286428), 7.109298)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.457204), 6.821732)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.647278), 6.656741)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.857006), 6.637899)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-1.085334), 6.809330)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.011208)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.177002), 8.082212)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.350324), 8.297457)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.516176), 8.662945)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.670722), 9.186354)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.810445), 9.873915)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.932785), 10.727701)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.011208)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.117196), 8.617856)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.218281), 9.326664)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.304451), 10.137327)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.377140), 11.050436)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.437970), 12.069004)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.488647), 13.201675)))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.011208)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 8.829264)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 9.714991)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.672543)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 11.708140)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 12.832425)),
                    entry(
                        6.000000,
                        new ShotResult(0.785398, new Rotation2d(-0.000000), 14.065099))))));
    put(
        8.45740850988178,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.349611)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.591850)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 7.892420)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 7.247412)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 6.653029)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 6.105489)),
                    entry(6.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 5.601025)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.349611)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.114509), 8.838698)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.241899), 8.420924)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.382564), 8.098234)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.536898), 7.876107)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.705263), 7.765856)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.887772), 7.788345)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.349611)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.152644), 9.411703)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.302975), 9.599700)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.448613), 9.918486)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.587106), 10.375260)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.716049), 10.978247)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.833353), 11.735488)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.349611)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.102060), 9.953856)),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.271095), 11.450144)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.339582), 12.350369)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.398657), 13.363382)),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.349611)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 10.169998)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 11.058252)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 12.021629)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 13.071425)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 14.227106)),
                    entry(6.000000, ShotResult.invalid)))));
    put(
        10.446760637352225,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.565717)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.796790)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.085937)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.428506)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 7.820429)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 7.257961)),
                    entry(6.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 6.737557)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.565717)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.101405), 10.044139)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.212805), 9.609425)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.334425), 9.261554)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.466483), 9.003012)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.609192), 8.839909)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.762703), 8.783792)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.565717)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.136188), 10.622665)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.270748), 10.794961)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.402016), 11.086858)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.528279), 11.505113)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.647825), 12.058607)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.759072), 12.758227)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.565717)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.091672), 11.176090)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.173947), 11.879015)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.247266), 12.680652)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.312202), 13.591281)),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.565717)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 11.398454)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 12.302766)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 13.290252)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 14.380096)),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid)))));
    put(
        12.436112764822669,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 11.698762)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.911872)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.185045)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.512352)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.889031)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.311016)),
                    entry(6.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 7.774681)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 11.698762)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.092049), 11.162604)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.192223), 10.710757)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.300659), 10.341627)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.417475), 10.055820)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.542776), 9.856639)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.676643), 9.751036)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 11.698762)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.124304), 11.752826)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.247381), 11.916361)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.367984), 12.193389)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.484843), 12.590642)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.596690), 13.117766)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.702337), 13.788131)))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 11.698762)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.084099), 12.322950)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.160490), 13.041498)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.229509), 13.864792)),
                    entry(4.000000, ShotResult.invalid),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 11.698762)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 12.553683)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 13.488303)),
                    entry(3.000000, ShotResult.invalid),
                    entry(4.000000, ShotResult.invalid),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid)))));
    put(
        14.425464892293114,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 12.775972)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 11.963098)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 11.214957)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.523545)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.882891)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.288239)),
                    entry(6.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 8.735595)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 12.775972)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.085063), 12.220034)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.176942), 11.748945)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.275725), 11.359398)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.381475), 11.050417)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.494234), 10.823416)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.614018), 10.682641)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 12.775972)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.115375), 12.828794)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.229785), 12.988623)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.342259), 13.259626)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.451819), 13.649085)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.557508), 14.168179)),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 12.775972)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.078383), 13.422848)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.150288), 14.170851)),
                    entry(3.000000, ShotResult.invalid),
                    entry(4.000000, ShotResult.invalid),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 12.775972)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.000000), 13.665187)),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, ShotResult.invalid),
                    entry(4.000000, ShotResult.invalid),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid)))));
    put(
        16.41481701976356,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 13.821052)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 12.971256)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 12.194706)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 11.479920)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.818935)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.205817)),
                    entry(6.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 9.635869)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 13.821052)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.079709), 13.237773)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.165254), 12.743260)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.256706), 12.331845)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.354100), 12.000804)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.457440), 11.749915)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.566706), 11.581511)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 13.821052)),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.108521), 13.874098)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.216268), 14.034768)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.322473), 14.307847)),
                    entry(4.000000, ShotResult.invalid),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 13.821052)),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, ShotResult.invalid),
                    entry(4.000000, ShotResult.invalid),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 13.821052)),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, ShotResult.invalid),
                    entry(4.000000, ShotResult.invalid),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid)))));
    put(
        18.404169147234004,
        new AngleEntry(
            entry(
                0.000000,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 13.956637)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 13.141379)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 12.396594)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 11.710939)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 11.076531)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(0.000000), 10.487506)))),
            entry(
                0.785398,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, new ShotResult(0.785398, new Rotation2d(-0.075559), 14.237474)),
                    entry(2.000000, new ShotResult(0.785398, new Rotation2d(-0.156176), 13.712309)),
                    entry(3.000000, new ShotResult(0.785398, new Rotation2d(-0.241927), 13.275113)),
                    entry(4.000000, new ShotResult(0.785398, new Rotation2d(-0.332839), 12.920704)),
                    entry(5.000000, new ShotResult(0.785398, new Rotation2d(-0.428902), 12.647029)),
                    entry(
                        6.000000, new ShotResult(0.785398, new Rotation2d(-0.530075), 12.454725)))),
            entry(
                1.570796,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, ShotResult.invalid),
                    entry(4.000000, ShotResult.invalid),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                2.356194,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, ShotResult.invalid),
                    entry(4.000000, ShotResult.invalid),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid))),
            entry(
                3.141593,
                new VelocityEntry(
                    entry(0.000000, ShotResult.invalid),
                    entry(1.000000, ShotResult.invalid),
                    entry(2.000000, ShotResult.invalid),
                    entry(3.000000, ShotResult.invalid),
                    entry(4.000000, ShotResult.invalid),
                    entry(5.000000, ShotResult.invalid),
                    entry(6.000000, ShotResult.invalid)))));
  }
}
