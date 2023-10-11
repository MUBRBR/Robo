import SmartArloNew

betterArlo = SmartArloNew.betterRobot()

# betterArlo.DriveLength(1)
# betterArlo.Stop()
# betterArlo.RotateAngle(90)

# manuually folowing obstacles course
betterArlo.Radar.active = False

# betterArlo.FollowRoute(True)

# betterArlo.RotateAngle(180)

betterArlo.AddDest([1,1])
betterArlo.AddDest([-1,1])
betterArlo.FollowRoute(True)

