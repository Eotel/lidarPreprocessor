#include "ofApp.h"

#include <sys/stat.h>

//--------------------------------------------------------------
void ofApp::setup()
{
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetFrameRate(60);

    ofxXmlSettings settings;
    settings.load("settings.xml");

    // ofParameters
    guiPanel.setup();
    guiPanel.setName("Settings");

    // OSC
    oscParams.setName("OSC");
    oscParams.add(oscSendIP.set("Send IP", settings.getValue("Settings:OSC:oscSendIP", "localhost")));
    oscParams.add(oscSendPort.set("Send Port", settings.getValue("Settings:OSC:oscSendPort", 10000), 0, 65535));
    oscParams.add(oscReceivePort.set("Receive Port", settings.getValue("Settings:OSC:oscReceivePort", 8000), 0, 65535));
    oscParams.add(sendCartesian.set("Send Cartesian", true));

    // LiDAR
    params.setName("LiDAR Parameters");
    params.add(yawOffset.set("Yaw Offset", 0, -180, 180));
    params.add(rollOffset.set("Roll Offset", 0, -90, 90));
    params.add(pitchOffset.set("Pitch Offset", 0, -90, 90));
    params.add(minDistance.set("Min Distance", 0, 0, 1000));
    params.add(maxDistance.set("Max Distance", 1000, 0, 30000)); // Adjusted max distance to 30,000 mm

    params.add(pointSize.set("Point Size", 2, 0.5, 10));
    params.add(pointAlpha.set("Point Alpha", 128, 0, 255));

    // Tracker
    trackerParams.setName("Tracker Parameters");
    trackerParams.add(minArea.set("Min Area", 10, 1, 100));
    trackerParams.add(maxArea.set("Max Area", 200, 100, 500));
    trackerParams.add(threshold.set("Threshold", 128, 0, 255));
    trackerParams.add(findHoles.set("Find Holes", false));
    trackerParams.add(persistence.set("Persistence", 15, 0, 100));
    trackerParams.add(maximumDistance.set("Max Distance", 32.0f, 1.0f, 100.0f));

    // View
    viewParams.setName("View Parameters");
    viewParams.add(showPCL.set("Show PCL", true));
    viewParams.add(showFBO.set("Show FBO", false));
    viewParams.add(showTracker.set("Show Tracker", false));
    viewParams.add(showGrid.set("Show Grid", true)); // Added grid visibility
    viewParams.add(scale.set("Scale", 0.01f, 0.001f, 1.0f)); // Added scaling parameter
    viewParams.add(gridSpacing.set("Grid Spacing", 500.0f, 100.0f, 5000.0f)); // Grid spacing parameter

    // Debug
    debugParams.setName("Debug Info");
    debugParams.add(fps.set("FPS", 0.0f));
    debugParams.add(blobInfo.set("Blob Sizes", ""));

    guiPanel.add(oscParams);
    guiPanel.add(params);
    guiPanel.add(trackerParams);
    guiPanel.add(viewParams);
    guiPanel.add(debugParams);

    guiPanel.add(loadButton.setup("load"));
    guiPanel.add(saveButton.setup("save"));

    loadSettings();

    loadButton.addListener(this, &ofApp::loadSettings);
    saveButton.addListener(this, &ofApp::saveSettings);
    ofAddListener(oscParams.parameterChangedE(), this, &ofApp::oscSetup);
    ofAddListener(trackerParams.parameterChangedE(), this, &ofApp::trackerSetup);
    ofAddListener(params.parameterChangedE(), this, &ofApp::updateRotationMatrix);

    oscSetup(oscParams);
    trackerSetup(trackerParams);
    updateRotationMatrix(params);

    fbo.allocate(ofGetWidth(), ofGetHeight(), GL_RGBA);

    pointMesh.setMode(OF_PRIMITIVE_POINTS);
    glPointSize(pointSize);

    glEnable(GL_POINT_SMOOTH);

    easyCam.setFarClip(100000.0f);
    easyCam.setNearClip(10.0f);
}

//--------------------------------------------------------------
void ofApp::update()
{
    // Process OSC messages
    ofxOscMessage m;
    while (receiver.hasWaitingMessages())
    {
        receiver.getNextMessage(m);
        if (m.getAddress() == "/lidar")
        {
            std::vector<glm::vec3> points;
            for (int i = 1; i < m.getNumArgs(); i += 2)
            {
                const float angle = m.getArgAsFloat(i);
                const float distance = m.getArgAsFloat(i + 1);
                glm::vec3 point(distance * cos(glm::radians(angle)),
                                distance * sin(glm::radians(angle)),
                                0);
                points.push_back(point);
            }
            // Process the points
            processPoints(points);

            // Update point mesh
            pointMesh.clear();
            pointMesh.addVertices(processedPoints);

            // Update FBO and tracking
            updateFbo();
            detectAndTrackBlobs();
            sendTrackedBlobs();
        }
    }

    // Update FPS
    fps = ofGetFrameRate();

    // Update blob information
    std::stringstream ss;
    for (int i = 0; i < contourFinder.size(); i++)
    {
        const float area = contourFinder.getContourArea(i);
        ss << "Blob " << i << " Area: " << area << "\n";
    }
    blobInfo = ss.str();

    // Enable or disable mouse input for easyCam based on GUI interaction
    if (showGUI && guiPanel.getShape().inside(ofGetMouseX(), ofGetMouseY()))
    {
        easyCam.disableMouseInput();
    }
    else
    {
        easyCam.enableMouseInput();
    }
}

void ofApp::drawGrid() const
{
    ofPushStyle();
    ofSetColor(50);
    const float gridLimit = maxDistance * 1.5f;

    // Draw grid in 3D space
    for (float x = -gridLimit; x <= gridLimit; x += gridSpacing)
    {
        ofDrawLine(x, -gridLimit, 0, x, gridLimit, 0); // Lines along Y-axis
        ofDrawLine(-gridLimit, x, 0, gridLimit, x, 0); // Lines along X-axis
    }
    ofPopStyle();
}

void ofApp::drawPCL()
{
    ofPushStyle();
    // Draw original point cloud in green
    ofSetColor(0, 255, 0, pointAlpha);
    glPointSize(pointSize);
    pointMesh.draw();

    // Draw projected point cloud in blue
    ofSetColor(0, 0, 255, pointAlpha);
    projectedMesh.setMode(OF_PRIMITIVE_POINTS);
    projectedMesh.addVertices(projectedPoints);
    projectedMesh.draw();
    ofPopStyle();
}

void ofApp::drawFBO()
{
    ofPushMatrix();
    ofTranslate(0, 0);

    if (showFBO)
    {
        fbo.draw(0, 0);
    }

    if (showTracker)
    {
        ofPushStyle();
        ofNoFill();
        ofSetColor(255, 0, 0);
        contourFinder.draw();

        const auto& tracker = contourFinder.getTracker();
        for (int i = 0; i < contourFinder.size(); i++)
        {
            const cv::Rect& rect = contourFinder.getBoundingRect(i);
            const cv::Point2f& centerCv = contourFinder.getCenter(i);
            glm::vec2 center = ofxCv::toOf(centerCv);

            int label = contourFinder.getLabel(i);
            std::string msg = ofToString(label) + ":" + ofToString(tracker.getAge(label));

            ofDrawBitmapString(msg, center.x, center.y);

            const glm::vec2 velocity = ofxCv::toOf(contourFinder.getVelocity(i));
            ofDrawLine(center, center + velocity * 5);
        }
        ofPopStyle();
    }

    ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofBackground(0);
    ofEnableDepthTest();

    easyCam.begin();

    ofScale(scale, scale, scale);

    if (showGrid) drawGrid();
    if (showPCL) drawPCL();

    easyCam.end();

    ofDisableDepthTest();

    if (showFBO || showTracker) drawFBO();
    if (showGUI) guiPanel.draw();
}


//--------------------------------------------------------------
void ofApp::exit()
{
    loadButton.removeListener(this, &ofApp::loadSettings);
    saveButton.removeListener(this, &ofApp::saveSettings);
    ofRemoveListener(oscParams.parameterChangedE(), this, &ofApp::oscSetup);
    ofRemoveListener(trackerParams.parameterChangedE(), this, &ofApp::trackerSetup);
    ofRemoveListener(params.parameterChangedE(), this, &ofApp::updateRotationMatrix);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    if (key == ' ')
    {
        showGUI = !showGUI;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseScrolled(int x, int y, float scrollX, float scrollY)
{
};

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y)
{
};

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y)
{
};

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
};

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
};

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
};

void ofApp::updateRotationMatrix(ofAbstractParameter& e)
{
    rotationMatrix = glm::mat4(1.0f);
    rotationMatrix = glm::rotate(rotationMatrix, glm::radians(yawOffset.get()), glm::vec3(0, 1, 0));
    rotationMatrix = glm::rotate(rotationMatrix, glm::radians(pitchOffset.get()), glm::vec3(1, 0, 0));
    rotationMatrix = glm::rotate(rotationMatrix, glm::radians(rollOffset.get()), glm::vec3(0, 0, 1));
}

glm::vec3 ofApp::rotatePoint(const glm::vec3& point, const glm::mat4& rotationMatrix)
{
    return glm::vec3(rotationMatrix * glm::vec4(point, 1.0f));
}

void ofApp::loadSettings()
{
    if (ofFile("settings.xml").exists())
    {
        guiPanel.loadFromFile("settings.xml");
    }
    else
    {
        ofLogError("ofApp::loadSettings") << "settings.xml does not exist";
        guiPanel.saveToFile("settings.xml");
    }
}

void ofApp::saveSettings()
{
    guiPanel.saveToFile("settings.xml");
}

void ofApp::oscSetup(ofAbstractParameter& e)
{
    if (e.getName() == "Send Cartesian") return;

    ofLogNotice("ofApp::oscChanged") << "Send IP: " << oscSendIP << ", Send Port: " << oscSendPort;
    ofLogNotice("ofApp::oscChanged") << "Receive Port: " << oscReceivePort;
    sender.setup(oscSendIP, oscSendPort);
    receiver.setup(oscReceivePort);
}

void ofApp::trackerSetup(ofAbstractParameter& e)
{
    contourFinder.setMinAreaRadius(minArea);
    contourFinder.setMaxAreaRadius(maxArea);
    contourFinder.setThreshold(threshold);
    contourFinder.setFindHoles(findHoles);
    contourFinder.getTracker().setPersistence(persistence);
    contourFinder.getTracker().setMaximumDistance(maximumDistance);
}

void ofApp::processPoints(const std::vector<glm::vec3>& points)
{
    processedPoints.clear();
    projectedPoints.clear();

    for (const auto& point : points)
    {
        const float distance = glm::length(point);
        if (distance < minDistance || distance > maxDistance)
        {
            continue;
        }

        // Rotate point using the rotation matrix
        glm::vec3 rotatedPoint = rotatePoint(point, rotationMatrix);
        processedPoints.push_back(rotatedPoint);

        // Project the rotated point onto the grid plane (z = 0)
        glm::vec3 projectedPoint = rotatedPoint;
        projectedPoint.z = 0;
        projectedPoints.push_back(projectedPoint);
    }
}

void ofApp::updateFbo() const
{
    fbo.begin();
    ofClear(0, 0, 0, 0);

    ofPushMatrix();
    ofTranslate(fbo.getWidth() / 2, fbo.getHeight() / 2);
    ofScale(scale, scale, scale);

    ofSetColor(255);
    ofFill();
    for (const auto& point : projectedPoints)
    {
        ofDrawCircle(point.x, point.y, pointSize);
    }

    ofPopMatrix();
    fbo.end();
}

void ofApp::detectAndTrackBlobs()
{
    ofPixels pixels;
    fbo.readToPixels(pixels);

    const cv::Mat mat = ofxCv::toCv(pixels);
    cv::Mat gray;
    cv::cvtColor(mat, gray, cv::COLOR_RGBA2GRAY);

    contourFinder.setThreshold(threshold);
    contourFinder.findContours(gray);
}

void ofApp::sendTrackedBlobs()
{
    for (int i = 0; i < contourFinder.size(); i++)
    {
        const int label = contourFinder.getLabel(i);

        // Get center in image coordinates
        glm::vec2 center;
        try
        {
            center = ofxCv::toOf(contourFinder.getCenter(i));
        }
        catch (...)
        {
            // If center cannot be obtained, skip this blob
            continue;
        }

        // Convert to world coordinates (mm units)
        center.x = (center.x - fbo.getWidth() / 2) / scale;
        center.y = (center.y - fbo.getHeight() / 2) / scale;

        ofxOscMessage m;
        if (sendCartesian)
        {
            m.setAddress("/blob/cartesian");
            m.addIntArg(label);
            m.addFloatArg(center.x); // X coordinate in mm
            m.addFloatArg(center.y); // Y coordinate in mm
        }
        else
        {
            m.setAddress("/blob/polar");
            const float distance = glm::length(center);
            const float angle = glm::degrees(atan2(center.y, center.x));
            m.addIntArg(label);
            m.addFloatArg(angle); // Angle in degrees
            m.addFloatArg(distance); // Distance in mm
        }
        sender.sendMessage(m, false); // Non-blocking send
    }
}
