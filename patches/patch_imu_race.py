import sys

PATH = "/bridge_ws/ardupilot_gazebo_src/src/ArduPilotPlugin.cc"

OLD_A = """    if (!this->dataPtr->imuInitialized)
    {
        // Set unconditionally because we're only going to try this once.
        this->dataPtr->imuInitialized = true;
        std::string imuTopicName;"""

NEW_A = """    if (!this->dataPtr->imuInitialized)
    {
        std::string imuTopicName;"""

OLD_B = """        else
        {
            gzerr << "[" << this->dataPtr->modelName << "] "
                  << "imu_sensor [" << this->dataPtr->imuName
                  << "] not found, abort ArduPilot plugin." << "\\n";
            return;
        }

        this->dataPtr->node.Subscribe(imuTopicName,"""

NEW_B = """        else
        {
            if (_info.simTime < std::chrono::seconds(10))
            {
                return;
            }
            this->dataPtr->imuInitialized = true;
            gzerr << "[" << this->dataPtr->modelName << "] "
                  << "imu_sensor [" << this->dataPtr->imuName
                  << "] not found, abort ArduPilot plugin." << "\\n";
            return;
        }

        this->dataPtr->imuInitialized = true;
        this->dataPtr->node.Subscribe(imuTopicName,"""

src = open(PATH).read()

for name, old in (("A", OLD_A), ("B", OLD_B)):
    n = src.count(old)
    if n != 1:
        print("ABORT: anchor %s matched %d times (need exactly 1)" % (name, n))
        sys.exit(1)

src = src.replace(OLD_A, NEW_A).replace(OLD_B, NEW_B)
open(PATH, "w").write(src)
print("OK: both edits applied")
