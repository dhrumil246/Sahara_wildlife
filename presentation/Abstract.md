# SAHARA: Human–Wildlife Early Warning — Abstract

**Problem statement:** PS000183 · AI-Powered Solutions for Mitigating Human–Wildlife Conflict

Human–wildlife conflict kills people and animals across India: about 2,860 people were killed by elephants between 2019 and 2024, and 392 elephants were electrocuted in five years. Forests already have AI camera towers and camera traps. But each camera sees only one spot, so once an elephant herd or a leopard walks out of view it is lost, and every first sighting either causes panic or is ignored.

SAHARA links these existing cameras into chains along the paths animals use toward villages. The Sentinel detector (YOLOv8 by day, thermal cameras at night) measures each animal's size on the ground, so cattle and dogs never raise an alert. Path logic reads the order in which cameras see an animal to work out its direction, speed and time to reach the village. A single sighting only puts the animal under watch. When the animal reaches the launch camera and its path toward the village is confirmed, an autonomous drone (S500, ArduCopter) takes off by itself. It flies to where the herd is heading, follows from 25 m behind without descending, and keeps the village ETA up to date. Officers are told first. Villagers are warned in English and Hindi through Telegram and a phone reporting page.

In a Gazebo and ArduPilot simulation, the system did not launch when a tusker turned back, launched at the right camera for a herd, tracked the herd with a median position error of 1.6 m, and raised no false elephant alerts. On real footage, the same detector found elephants in 235 of 235 frames. The software is open source and runs offline on one laptop. Next steps are training on Indian camera-trap data, connecting real towers, and a field trial.
