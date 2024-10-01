#include "ofApp.h"

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

    oscSetup(oscParams);
    trackerSetup(trackerParams);

    ofLogNotice() << "ofApp::setup";

    fbo.allocate(ofGetWidth(), ofGetHeight());

    pointMesh.setMode(OF_PRIMITIVE_POINTS);
    glPointSize(pointSize);

    glEnable(GL_POINT_SMOOTH);

    easyCam.setFarClip(100000.0f);
    easyCam.setNearClip(10.0f);


    // syphonServer.setName("FBO Output");
}

//--------------------------------------------------------------
void ofApp::update()
{
    // Receive OSC messages
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
                glm::vec3 point(distance * cos(ofDegToRad(angle)),
                                distance * sin(ofDegToRad(angle)),
                                0);
                points.push_back(point);
            }
            // Process the points directly
            processPoints(points);

            // Update point mesh
            pointMesh.clear();
            pointMesh.addVertices(processedPoints);

            // Continue with your update logic
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
    const float gridLimit = maxDistance * 1.5f; // Extend the grid beyond the max distance

    // Draw grid in 3D space
    for (float x = -gridLimit; x <= gridLimit; x += gridSpacing)
    {
        ofDrawLine(x, -gridLimit, 0, x, gridLimit, 0); // Lines along Y-axis
        ofDrawLine(-gridLimit, x, 0, gridLimit, x, 0); // Lines along X-axis
    }
    ofPopStyle();
}

void ofApp::drawPCL() const
{
    ofPushStyle();
    ofSetColor(0, 255, 0, pointAlpha); // Set point color and alpha
    glPointSize(pointSize); // Update point size
    pointMesh.draw(); // Draw point mesh
    ofPopStyle();
}

void ofApp::drawFBO()
{
    ofPushMatrix();
    ofTranslate(ofGetWidth() / 2 - fbo.getWidth() / 2, ofGetHeight() / 2 - fbo.getHeight() / 2);

    if (showFBO)
    {
        fbo.draw(0, 0);
    }

    if (showTracker)
    {
        contourFinder.draw();

        const ofxCv::RectTracker& tracker = contourFinder.getTracker();
        for (int i = 0; i < contourFinder.size(); i++)
        {
            const ofRectangle r = ofxCv::toOf(contourFinder.getBoundingRect(i));
            const ofPoint center = ofxCv::toOf(contourFinder.getCenter(i));
            ofPushMatrix();
            ofTranslate(center.x, center.y);
            int label = contourFinder.getLabel(i);
            string msg = ofToString(label) + ":" + ofToString(tracker.getAge(label));
            ofDrawBitmapString(msg, 0, 0);
            const ofVec2f velocity = ofxCv::toOf(contourFinder.getVelocity(i));
            ofScale(5, 5);
            ofDrawLine(0, 0, velocity.x, velocity.y);
            ofPopMatrix();
        }
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


ofVec3f ofApp::rotatePoint(const ofVec3f& point, const float roll, const float pitch, const float yaw)
{
    // Create quaternions for each rotation
    const ofQuaternion qPitch(pitch, ofVec3f(1, 0, 0));
    const ofQuaternion qYaw(yaw, ofVec3f(0, 1, 0));
    const ofQuaternion qRoll(roll, ofVec3f(0, 0, 1));

    const ofQuaternion rotation = qPitch * qYaw * qRoll;

    const ofVec3f rotatedPoint = rotation * point;

    return rotatedPoint;
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
    for (const auto& point : points)
    {
        if (const float distance = glm::length(point); distance < minDistance || distance > maxDistance)
        {
            continue;
        }

        ofVec3f rotatedPoint = rotatePoint(point, rollOffset, pitchOffset, yawOffset);

        processedPoints.push_back(rotatedPoint);
    }
}

void ofApp::updateFbo() const
{
    fbo.begin();
    ofClear(0, 0, 0, 255);

    ofPushMatrix();
    ofTranslate(fbo.getWidth() / 2, fbo.getHeight() / 2);

    ofScale(scale, scale, scale);

    ofSetColor(pointAlpha);
    ofEnableBlendMode(OF_BLENDMODE_ADD); // Enable additive blending
    for (const auto& point : processedPoints)
    {
        // Project the 3D point onto the z=0 plane for FBO rendering
        ofPoint projectedPoint = point;
        projectedPoint.z = 0;

        ofDrawCircle(projectedPoint.x, projectedPoint.y, pointSize);
    }
    ofDisableBlendMode();

    ofPopMatrix();
    fbo.end();
}

void ofApp::detectAndTrackBlobs()
{
    ofPixels pixels;
    fbo.readToPixels(pixels);

    const cv::Mat mat = ofxCv::toCv(pixels);
    cv::Mat gray;
    cv::cvtColor(mat, gray, cv::COLOR_RGB2GRAY);

    contourFinder.findContours(gray);
}

void ofApp::sendTrackedBlobs()
{
    ofxOscMessage m;
    m.setAddress("/tracked_blobs");

    for (int i = 0; i < contourFinder.size(); i++)
    {
        const ofRectangle boundingRect = ofxCv::toOf(contourFinder.getBoundingRect(i));
        const ofPoint center = ofxCv::toOf(contourFinder.getCenter(i));
        const ofVec2f velocity = ofxCv::toOf(contourFinder.getVelocity(i));
        const int label = contourFinder.getLabel(i);

        m.addIntArg(label);
        m.addFloatArg(center.x);
        m.addFloatArg(center.y);
        m.addFloatArg(boundingRect.width);
        m.addFloatArg(boundingRect.height);
        m.addFloatArg(velocity.x);
        m.addFloatArg(velocity.y);
    }

    sender.sendMessage(m);
}
