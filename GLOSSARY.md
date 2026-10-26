# Receptionist Bot

A TurtleBot4 that waits at a reception, notices people walking up, talks to them, and can guide them to places on the map.

## Language

**Visitor**:
A person who has walked up to the robot and is treated as someone to talk to.
_Avoid_: Guest, person, user, customer

**Visitor candidate**:
Something in front of the robot that looks like a person to the sensors, but hasn't been there long enough to count as a **Visitor**.
_Avoid_: Detection, cluster, person

**Visit**:
One whole interaction with a **Visitor**, from noticing them until the robot is ready at the reception again, including any guiding drive and the drive back.
_Avoid_: Session, conversation, tour

**Location**:
A saved, named pose on the map (position and heading) the robot can drive to.
_Avoid_: Waypoint, exhibit, goal, target
