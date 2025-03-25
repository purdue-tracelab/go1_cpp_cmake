#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <tinyxml2.h>
#include <Eigen/Dense>

#include "go1_cpp_cmake/go1Utils.h"

struct Inertia {
    double mass;
    Eigen::Vector3d diagInertia;
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
};

struct Joint { // no offsets from link frame
    std::string name;
    std::string parentLink;
    double angle;
    Eigen::Vector3d axis;
};

struct Link {
    Eigen::Vector3d parentPos;
    Inertia inertia;
    Joint parentJoint;
};

void parseLink(tinyxml2::XMLElement* bodyElement, std::map<std::string, Link>& linkMap) {
    Link link;
    std::string linkName = bodyElement->Attribute("name"); // key for totalLinkMap
    tinyxml2::XMLElement* inertialData = bodyElement->FirstChildElement("inertial");

    const char* parentPosText = bodyElement->Attribute("pos");
    std::istringstream(parentPosText) >> link.parentPos.x() >> link.parentPos.y() >> link.parentPos.z(); 

    // Extract mass, inertia, and inertial frame pose
    const char* massText = inertialData->Attribute("mass");
    const char* diagInertiaText = inertialData->Attribute("diaginertia");
    const char* posText = inertialData->Attribute("pos");
    const char* quatText = inertialData->Attribute("quat");

    link.inertia.mass = std::stod(massText);
    std::istringstream(diagInertiaText) >> link.inertia.diagInertia.x() >> link.inertia.diagInertia.y() >> link.inertia.diagInertia.z();
    std::istringstream(posText) >> link.inertia.pos.x() >> link.inertia.pos.y() >> link.inertia.pos.z();
    std::istringstream(quatText) >> link.inertia.quat.w() >> link.inertia.quat.x() >> link.inertia.quat.y() >> link.inertia.quat.z();

    tinyxml2::XMLElement* jointData = bodyElement->FirstChildElement("joint");

    if (jointData) {
        link.parentJoint.name = jointData->Attribute("name");

        // Extract joint axis
        const char* classText = jointData->Attribute("class");
        if (classText) {
            std::string classStr(classText); // convert pointer reference to string

            if (classStr == "hip") {
                link.parentJoint.axis << 0, 1, 0;
            } else if (classStr == "abduction") {
                link.parentJoint.axis << 1, 0, 0;
            } else if (classStr == "knee") {
                link.parentJoint.axis << 0, 1, 0;
            }
        }

        link.parentJoint.angle = 0; // set zero for now, will set in keyframe or int main() function

    } else { // only trunk without a parent "Joint" is trunk with a parent free joint a.k.a floating
        link.parentJoint.name = "floating";
        link.parentJoint.angle = 0;
        link.parentJoint.axis << 0, 0, 0;
    }

    linkMap[linkName] = link;

    // Parse any child links if present
    for (tinyxml2::XMLElement* childBody = bodyElement->FirstChildElement("body"); childBody != nullptr; childBody = childBody->NextSiblingElement("body")) {
        parseLink(childBody, linkMap);
        linkMap[childBody->Attribute("name")].parentJoint.parentLink = linkName;
    }

}

std::map<std::string, Link> parseXML(const std::string& xmlFile) {
    tinyxml2::XMLDocument doc;
    doc.LoadFile(xmlFile.c_str());
    tinyxml2::XMLElement* root = doc.RootElement(); // root element is <mujoco>
    tinyxml2::XMLElement* worldbodyElement = root->FirstChildElement("worldbody"); // access <worldbody> element

    std::map<std::string, Link> totalLinkMap;
    std::map<std::string, double> jointOrder;

    // Parse the bodies starting from <worldbody>
    for (tinyxml2::XMLElement* body = worldbodyElement->FirstChildElement("body"); body != nullptr; body = body->NextSiblingElement("body")) {
        parseLink(body, totalLinkMap);  // Recursively parse all nested bodies and joints
    }

    // Parse the joint positions in order from <keyframe>
    tinyxml2::XMLElement* keyElement = root->FirstChildElement("keyframe")->FirstChildElement("key");
    const char* qposText = keyElement->Attribute("qpos");

    std::istringstream qposStream(qposText);
    std::vector<double> qpos_vals;
    double jointPos;

    while (qposStream >> jointPos) {
        qpos_vals.push_back(jointPos);
    }

    // Populate the joint positions with the qpos vector in <keyframe> w/ the order in <actuator>
    tinyxml2::XMLElement* actuatorElement = root->FirstChildElement("actuator");
    double qpos_idx = 7; // after root position and quat in qpos vec
    for (tinyxml2::XMLElement* motor = actuatorElement->FirstChildElement("motor"); motor != nullptr; motor = motor->NextSiblingElement("motor")) {
        std::string motorLink = motor->Attribute("name");
        totalLinkMap[motorLink].parentJoint.angle = qpos_vals[qpos_idx];
        qpos_idx++;
    }

    return totalLinkMap;
}

void printLinkMap(std::map<std::string, Link>& linkMap) {
    for (const auto& link : linkMap) {
        std::cout << "Link: " << link.first << "\n";
        std::cout << "  Mass: " << link.second.inertia.mass << "\n";
        std::cout << "  Inertia:\n";
        std::cout << "      Ixx: " << link.second.inertia.diagInertia.x() << "\n";
        std::cout << "      Iyy: " << link.second.inertia.diagInertia.y() << "\n";
        std::cout << "      Izz: " << link.second.inertia.diagInertia.z() << "\n";
        std::cout << "  Inertial frame pos: " << link.second.inertia.pos.transpose() << "\n";
        std::cout << "  Inertial frame quat: " << link.second.inertia.quat.w() << ", " << link.second.inertia.quat.x() << ", " << link.second.inertia.quat.y() << ", " << link.second.inertia.quat.z() << std::endl;
        std::cout << "  Reference frame pos: " << link.second.parentPos.x() << ", " << link.second.parentPos.y() << ", "<< link.second.parentPos.z() << "\n";
        std::cout << "  Parent joint: " << link.second.parentJoint.name << "\n";
        std::cout << "  Parent joint axis: " << link.second.parentJoint.axis.x() << ", " << link.second.parentJoint.axis.y() << ", " << link.second.parentJoint.axis.z() << "\n";
        std::cout << "  Parent joint angle: " << link.second.parentJoint.angle << " rad\n";
        std::cout << "  Parent link: " << link.second.parentJoint.parentLink << std::endl;
        std::cout << "########################" << std::endl;
    }
}

void transformInertiaTrunkFrame(Eigen::Matrix3d& tempInertiaMat, double& tempMass, std::map<std::string, Link>& linkMap, std::string linkName, bool transformSelf = true) {
    if (transformSelf) {
        // fetch inertia in inertial frame and transform from body frame to inertial frame
        Eigen::Vector3d tempPos = linkMap[linkName].inertia.pos;
        Eigen::Quaterniond tempQuat = linkMap[linkName].inertia.quat;
        tempMass = linkMap[linkName].inertia.mass;
        tempInertiaMat = linkMap[linkName].inertia.diagInertia.asDiagonal();

        // undo transformation so inertia is represented in link's body frame
        Eigen::Matrix3d tempRotMat = tempQuat.toRotationMatrix();
        tempInertiaMat = tempRotMat.transpose() * tempInertiaMat * tempRotMat; // undoes rotation
        tempInertiaMat = tempInertiaMat - tempMass*skew(-tempPos)*skew(-tempPos); // undoes translation

    }

    if (linkName != "trunk") { // fetch joint angle and offset from parent link body frame, and recursively transform inertia
        Eigen::Vector3d tempPosBody = linkMap[linkName].parentPos;
        double tempAngle = linkMap[linkName].parentJoint.angle;
        Eigen::Vector3d tempAxis = linkMap[linkName].parentJoint.axis;

        if (tempAxis == Eigen::Vector3d(1, 0, 0)) {
            tempInertiaMat = rotX(tempAngle).transpose() * tempInertiaMat * rotX(tempAngle);
        } else if (tempAxis == Eigen::Vector3d(0, 1, 0)) {
            tempInertiaMat = rotY(tempAngle).transpose() * tempInertiaMat * rotY(tempAngle);
        } else {
            tempInertiaMat = rotZ(tempAngle).transpose() * tempInertiaMat * rotZ(tempAngle);
        }

        tempInertiaMat = tempInertiaMat - tempMass*skew(-tempPosBody)*skew(-tempPosBody);

        // recursively call transformInertiaTrunkFrame with transformSelf = false
        transformInertiaTrunkFrame(tempInertiaMat, tempMass, linkMap, linkMap[linkName].parentJoint.parentLink, false);
    }

}

int main() {
    std::string xmlFile = "/home/memento/go1_cpp_cmake/models/go1.xml"; // remember to make this relative in the end
    std::map<std::string, Link> totalLinkMap = parseXML(xmlFile);

    // Print all links and joints
    std::cout << "### Links of go1.xml ###\n";
    printLinkMap(totalLinkMap);

    // Calculate trunk inertia in trunk body frame
    Eigen::Matrix3d trunkInertia = Eigen::Matrix3d::Zero();
    double trunkMass = 0;
    transformInertiaTrunkFrame(trunkInertia, trunkMass, totalLinkMap, "trunk", true);
    std::cout << "\nTrunk inertia:\n" << trunkInertia << "\nTrunk mass: " << trunkMass << std::endl;


    //// FR ////
    std::cout << "\n### FR inertias ###" << std::endl;
    // Calculate FR hip inertia in trunk body frame
    Eigen::Matrix3d hipFRInertia = Eigen::Matrix3d::Zero();
    double hipFRMass = 0;
    transformInertiaTrunkFrame(hipFRInertia, hipFRMass, totalLinkMap, "FR_hip", true);
    std::cout << "\nFR hip inertia:\n" << hipFRInertia << "\nFR hip mass: " << hipFRMass << std::endl;

    // Calculate FR thigh inertia in trunk body frame
    Eigen::Matrix3d thighFRInertia = Eigen::Matrix3d::Zero();
    double thighFRMass = 0;
    transformInertiaTrunkFrame(thighFRInertia, thighFRMass, totalLinkMap, "FR_thigh", true);
    std::cout << "\nFR thigh inertia:\n" << thighFRInertia << "\nFR thigh mass: " << thighFRMass << std::endl;

    // Calculate FR calf inertia in trunk body frame
    Eigen::Matrix3d calfFRInertia = Eigen::Matrix3d::Zero();
    double calfFRMass = 0;
    transformInertiaTrunkFrame(calfFRInertia, calfFRMass, totalLinkMap, "FR_calf", true);
    std::cout << "\nFR calf inertia:\n" << calfFRInertia << "\nFR calf mass: " << calfFRMass << std::endl;


    //// FL ////
    std::cout << "\n### FL inertias ###" << std::endl;
    // Calculate FL hip inertia in trunk body frame
    Eigen::Matrix3d hipFLInertia = Eigen::Matrix3d::Zero();
    double hipFLMass = 0;
    transformInertiaTrunkFrame(hipFLInertia, hipFLMass, totalLinkMap, "FL_hip", true);
    std::cout << "\nFL hip inertia:\n" << hipFLInertia << "\nFL hip mass: " << hipFLMass << std::endl;

    // Calculate FL thigh inertia in trunk body frame
    Eigen::Matrix3d thighFLInertia = Eigen::Matrix3d::Zero();
    double thighFLMass = 0;
    transformInertiaTrunkFrame(thighFLInertia, thighFLMass, totalLinkMap, "FL_thigh", true);
    std::cout << "\nFL thigh inertia:\n" << thighFLInertia << "\nFL thigh mass: " << thighFLMass << std::endl;

    // Calculate FL calf inertia in trunk body frame
    Eigen::Matrix3d calfFLInertia = Eigen::Matrix3d::Zero();
    double calfFLMass = 0;
    transformInertiaTrunkFrame(calfFLInertia, calfFLMass, totalLinkMap, "FL_calf", true);
    std::cout << "\nFL calf inertia:\n" << calfFLInertia << "\nFL calf mass: " << calfFLMass << std::endl;


    //// RR ////
    std::cout << "\n### RR inertias ###" << std::endl;
    // Calculate RR hip inertia in trunk body frame
    Eigen::Matrix3d hipRRInertia = Eigen::Matrix3d::Zero();
    double hipRRMass = 0;
    transformInertiaTrunkFrame(hipRRInertia, hipRRMass, totalLinkMap, "RR_hip", true);
    std::cout << "\nRR hip inertia:\n" << hipRRInertia << "\nRR hip mass: " << hipRRMass << std::endl;

    // Calculate RR thigh inertia in trunk body frame
    Eigen::Matrix3d thighRRInertia = Eigen::Matrix3d::Zero();
    double thighRRMass = 0;
    transformInertiaTrunkFrame(thighRRInertia, thighRRMass, totalLinkMap, "RR_thigh", true);
    std::cout << "\nRR thigh inertia:\n" << thighRRInertia << "\nRR thigh mass: " << thighRRMass << std::endl;

    // Calculate RR calf inertia in trunk body frame
    Eigen::Matrix3d calfRRInertia = Eigen::Matrix3d::Zero();
    double calfRRMass = 0;
    transformInertiaTrunkFrame(calfRRInertia, calfRRMass, totalLinkMap, "RR_calf", true);
    std::cout << "\nRR calf inertia:\n" << calfRRInertia << "\nRR calf mass: " << calfRRMass << std::endl;


    //// RL ////
    std::cout << "\n### RL inertias ###" << std::endl;
    // Calculate RL hip inertia in trunk body frame
    Eigen::Matrix3d hipRLInertia = Eigen::Matrix3d::Zero();
    double hipRLMass = 0;
    transformInertiaTrunkFrame(hipRLInertia, hipRLMass, totalLinkMap, "RL_hip", true);
    std::cout << "\nRL hip inertia:\n" << hipRLInertia << "\nRL hip mass: " << hipRLMass << std::endl;

    // Calculate RL thigh inertia in trunk body frame
    Eigen::Matrix3d thighRLInertia = Eigen::Matrix3d::Zero();
    double thighRLMass = 0;
    transformInertiaTrunkFrame(thighRLInertia, thighRLMass, totalLinkMap, "RL_thigh", true);
    std::cout << "\nRL thigh inertia:\n" << thighRLInertia << "\nRL thigh mass: " << thighRLMass << std::endl;

    // Calculate RL calf inertia in trunk body frame
    Eigen::Matrix3d calfRLInertia = Eigen::Matrix3d::Zero();
    double calfRLMass = 0;
    transformInertiaTrunkFrame(calfRLInertia, calfRLMass, totalLinkMap, "RL_calf", true);
    std::cout << "\nRL calf inertia:\n" << calfRLInertia << "\nRL calf mass: " << calfRLMass << std::endl;
    

    // Calculate total inertia and mass in trunk body frame
    double totalMass = 0;
    Eigen::Matrix3d totalInertia = Eigen::Matrix3d::Zero();

    totalMass += trunkMass + hipFRMass + thighFRMass + calfFRMass;
    totalMass += hipFLMass + thighFLMass + calfFLMass;
    totalMass += hipRRMass + thighRRMass + calfRRMass;
    totalMass += hipRLMass + thighRLMass + calfRLMass;

    totalInertia += trunkInertia + hipFRInertia + thighFRInertia; // + calfFRInertia;
    totalInertia += hipFLInertia + thighFLInertia; // + calfFLInertia;
    totalInertia += hipRRInertia + thighRRInertia; // + calfRRInertia;
    totalInertia += hipRLInertia + thighRLInertia; // + calfRLInertia;

    std::cout << "\n### Total mass: " << totalMass << " kg ###" << std::endl;
    std::cout << "\n### Total inertia in trunk frame ###\n" << totalInertia << std::endl;

    return 0;
}
