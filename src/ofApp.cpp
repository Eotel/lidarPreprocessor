#include "ofApp.h"
#include "ofxXmlSettings.h"


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
    params.add(rollOffset.set("Roll Offset", 0, -180, 180));
    params.add(pitchOffset.set("Pitch Offset", 0, -180, 180));
    params.add(xOffset.set("X Offset", 0, -1000, 1000));
    params.add(yOffset.set("Y Offset", 0, -1000, 1000));
    params.add(zOffset.set("Z Offset", 0, -1000, 1000));
    params.add(minDistance.set("Min Distance", 0, 0, 3000));
    params.add(maxDistance.set("Max Distance", 1000, 0, 30000)); // Adjusted max distance to 30,000 mm

    params.add(pointSize.set("Point Size", 2, 0.5, 500));
    params.add(pointAlpha.set("Point Alpha", 128, 0, 255));

    params.add(pointSizeMode.set("Point Size Mode", 0, 0, 2));
    params.add(pointSizeMultiplier.set("Point Size Multiplier", 1, 0.001, 2));
    params.add(floorMounted.set("Floor Mounted", true));

    // Tracker
    trackerParams.setName("Tracker Parameters");
    trackerParams.add(minArea.set("Min Area", 10, 1, 500));
    trackerParams.add(maxArea.set("Max Area", 200, 100, 1000));
    trackerParams.add(threshold.set("Threshold", 128, 0, 255));
    trackerParams.add(findHoles.set("Find Holes", false));
    trackerParams.add(persistence.set("Persistence", 15, 0, 240));
    trackerParams.add(maximumDistance.set("Max Distance", 32.0f, 1.0f, 1000.0f));

    // View
    viewParams.setName("View Parameters");
    viewParams.add(showPCL.set("Show PCL", true));
    viewParams.add(showFBO.set("Show FBO", false));
    viewParams.add(showTracker.set("Show Tracker", false));
    viewParams.add(showGrid.set("Show Grid", true));
    viewParams.add(showFBOGrid.set("Show FBO Grid", false));
    viewParams.add(scale.set("Scale", 0.01f, 0.001f, 1.0f));
    viewParams.add(gridSpacing.set("Grid Spacing", 500.0f, 100.0f, 5000.0f));
    viewParams.add(enableMask.set("Enable Mask", true));

    // Mask Parameters
    initializeMask();

    // MeshWarp setup
    ofRectangle rect(0, 0, ofGetWidth(), ofGetHeight());
    meshWarp.setup(rect, 4, 4);
    meshWarp.setUVRect(ofRectangle(0, 0, ofGetWidth(), ofGetHeight()));

    // Add MeshWarp parameters to GUI
    viewParams.add(showMeshWarp.set("Show MeshWarp", false));
    viewParams.add(meshWarpCols.set("MeshWarp Columns", 4, 2, 20));
    viewParams.add(meshWarpRows.set("MeshWarp Rows", 4, 2, 20));

    // Debug
    debugParams.setName("Debug Info");
    debugParams.add(fps.set("FPS", 0.0f));
    debugParams.add(blobInfo.set("Blob Sizes", ""));

    // Reset
    resetParams.setName("Reset Parameters");
    resetParams.add(resetTimeout.set("Reset Timeout", 5.0f, 0.1f, 30.0f));
    resetParams.add(fboDecayRate.set("FBO Decay Rate", 0.95f, 0.8f, 1.0f));

    guiPanel.add(oscParams);
    guiPanel.add(params);
    guiPanel.add(trackerParams);
    guiPanel.add(viewParams);
    guiPanel.add(maskParams);
    guiPanel.add(circleMaskParams);
    guiPanel.add(resetParams);
    guiPanel.add(debugParams);

    guiPanel.add(loadButton.setup("load"));
    guiPanel.add(saveButton.setup("save"));

    loadSettings();
    updateMaskPath();

    loadButton.addListener(this, &ofApp::loadSettings);
    saveButton.addListener(this, &ofApp::saveSettings);
    ofAddListener(oscParams.parameterChangedE(), this, &ofApp::oscSetup);
    ofAddListener(trackerParams.parameterChangedE(), this, &ofApp::trackerSetup);
    ofAddListener(params.parameterChangedE(), this, &ofApp::updateRotationMatrix);
    ofAddListener(maskParams.parameterChangedE(), this, &ofApp::updateMaskPath);

    guiPanel.minimizeAll();

    oscSetup(oscParams);
    trackerSetup(trackerParams);
    updateRotationMatrix(params);

    fbo.allocate(ofGetWidth(), ofGetHeight(), GL_RGBA);
    previousFbo.allocate(ofGetWidth(), ofGetHeight(), GL_RGBA);
    initializeFbo();

    pointMesh.setMode(OF_PRIMITIVE_POINTS);
    glPointSize(pointSize);

    glEnable(GL_POINT_SMOOTH);

    easyCam.setFarClip(100000.0f);
    easyCam.setNearClip(10.0f);

    selectedVertex = -1;
    isDragging     = false;

    lastOscReceiveTime = ofGetElapsedTimef();
}

void ofApp::initializeFbo() const
{
    fbo.begin();
    ofClear(0, 0, 0, 0);
    fbo.end();
    previousFbo.begin();
    ofClear(0, 0, 0, 0);
    previousFbo.end();
}

//--------------------------------------------------------------
void ofApp::update()
{
    ofWidth  = ofGetWidth();
    ofHeight = ofGetHeight();

    // Update MeshWarp grid if necessary
    if (meshWarpCols != meshWarp.getDivX() || meshWarpRows != meshWarp.getDivY())
    {
        ofRectangle rect(0, 0, ofWidth, ofHeight);
        meshWarp.setup(rect, meshWarpCols, meshWarpRows);
        meshWarp.setUVRect(ofRectangle(0, 0, ofGetWidth(), ofGetHeight()));
    }

    checkResetFbo();

    // Process OSC messages
    ofxOscMessage m;
    while (receiver.hasWaitingMessages())
    {
        receiver.getNextMessage(m);
        if (m.getAddress() == "/lidar")
        {
            lastOscReceiveTime = ofGetElapsedTimef(); // Update last receive time
            shouldResetFbo     = false;

            if (m.getNumArgs() < 3)
            {
                // Not enough arguments, skip
                continue;
            }
            const auto             lidarID = m.getArgAsInt(0);
            std::vector<glm::vec3> points;
            for (auto i = 1; i < m.getNumArgs(); i += 2)
            {
                const auto angle    = m.getArgAsFloat(i);
                const auto distance = m.getArgAsFloat(i + 1);
                glm::vec3  point(distance * cos(glm::radians(angle)),
                                distance * sin(glm::radians(angle)),
                                0);
                points.push_back(point);
            }
            processPoints(points);

            pointMesh.clear();
            projectedMesh.clear();
            pointMesh.addVertices(processedPoints);
        }
    }

    if (!shouldResetFbo)
    {
        updateFbo();
        detectAndTrackBlobs();
        sendTrackedBlobs();
    }

    // Update FPS
    fps = static_cast<int>(ofGetFrameRate());

    // Update blob information
    blobInfo = ofToString(contourFinder.size());

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
    const float gridLimit    = maxDistance * 1.5f;
    const int   numGridLines = static_cast<int>(gridLimit / gridSpacing);

    for (int i = -numGridLines; i <= numGridLines; ++i)
    {
        const float x = i * gridSpacing;
        ofDrawLine(x, -gridLimit, 0, x, gridLimit, 0); // Lines along Y-axis
        ofDrawLine(-gridLimit, x, 0, gridLimit, x, 0); // Lines along X-axis
    }
    ofPopStyle();
}

void ofApp::drawFBOGrid() const
{
    ofPushStyle();
    ofSetColor(50);
    const float gridLimit    = maxDistance * 1.5f;
    const int   numGridLines = static_cast<int>(gridLimit / gridSpacing);

    for (int i = -numGridLines; i <= numGridLines; ++i)
    {
        const float x = i * gridSpacing;
        ofDrawLine(x, -gridLimit, x, gridLimit); // Vertical lines
        ofDrawLine(-gridLimit, x, gridLimit, x); // Horizontal lines
    }
    ofPopStyle();
}

void ofApp::drawFBOGridLabels() const
{
    ofPushMatrix();

    const float fboDrawWidth  = fbo.getWidth();
    const float fboDrawHeight = fbo.getHeight();
    const float scaleFactor   = std::min(ofWidth / fboDrawWidth, ofHeight / fboDrawHeight);

    ofTranslate((ofWidth - fboDrawWidth * scaleFactor) / 2, (ofHeight - fboDrawHeight * scaleFactor) / 2);
    ofScale(scaleFactor, scaleFactor);

    // Match coordinate system inside FBO
    ofPushMatrix();
    ofTranslate(fbo.getWidth() / 2, fbo.getHeight() / 2);
    ofScale(scale, -scale);

    ofSetColor(255);
    constexpr float labelSpacing = 1000.0f; // 1 meter in mm
    const float     gridLimit    = maxDistance * 1.5f;
    const int       numLabels    = static_cast<int>(gridLimit / labelSpacing);

    for (int i = -numLabels; i <= numLabels; ++i)
    {
        if (i == 0) continue; // Skip origin
        const float pos   = i * labelSpacing;
        std::string label = std::to_string(i) + "m";

        // Draw labels at (pos, 0) and (0, pos)
        ofDrawBitmapString(label, pos + 5, 15);
        ofDrawBitmapString(label, 5, pos + 15);
    }

    ofPopMatrix();
    ofPopMatrix();
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

void ofApp::drawFBO() const
{
    ofPushMatrix();

    // Center and scale the FBO
    const auto fboDrawWidth  = fbo.getWidth();
    const auto fboDrawHeight = fbo.getHeight();

    const auto scaleFactor = std::min(ofWidth / fboDrawWidth, ofHeight / fboDrawHeight);

    ofTranslate((ofWidth - fboDrawWidth * scaleFactor) / 2, (ofHeight - fboDrawHeight * scaleFactor) / 2);
    ofScale(scaleFactor, scaleFactor);

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

        for (auto i = 0; i < contourFinder.size(); i++)
        {
            const cv::Rect&    rect     = contourFinder.getBoundingRect(i);
            const cv::Point2f& centerCv = contourFinder.getCenter(i);
            const glm::vec2    center   = ofxCv::toOf(centerCv);

            int label = contourFinder.getLabel(i);

            ofDrawBitmapString(ofToString(label), center.x, center.y);
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
    if (showFBOGrid) drawFBOGridLabels();
    if (showGUI) guiPanel.draw();

    // マスクのインタラクション用表示(多角形マスク)
    if (enableMask)
    {
        drawMask();
        drawCircleMasks();
    }
}


//--------------------------------------------------------------
void ofApp::exit()
{
    loadButton.removeListener(this, &ofApp::loadSettings);
    saveButton.removeListener(this, &ofApp::saveSettings);
    ofRemoveListener(oscParams.parameterChangedE(), this, &ofApp::oscSetup);
    ofRemoveListener(trackerParams.parameterChangedE(), this, &ofApp::trackerSetup);
    ofRemoveListener(params.parameterChangedE(), this, &ofApp::updateRotationMatrix);
    ofRemoveListener(maskParams.parameterChangedE(), this, &ofApp::updateMaskPath);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    if (key == ' ')
    {
        showGUI = !showGUI;
    }
    else if (key == 'm')
    {
        // マウス位置に新しい円マスクを追加
        // screen座標を取得
        const auto mx = ofGetMouseX();
        const auto my = ofGetMouseY();

        const glm::vec2 worldPos = screenToWorld(mx, my);
        addCircleMask(worldPos);
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
    if (showGUI && guiPanel.getShape().inside(x, y)) return;

    // 既存の多角形マスク頂点選択処理
    for (auto i = 0; i < maskVerticesParams.size(); i++)
    {
        if (glm::vec2 screenPos = worldToScreen(maskVerticesParams[i].get());
            glm::distance(glm::vec2(x, y), screenPos) < 15.0f)
        {
            selectedVertex = i;
            isDragging     = true;
            return;
        }
    }

    // 円形マスクヒットテスト
    if (const auto hitIndex = hitTestCircleMaskScreen(x, y); hitIndex >= 0)
    {
        selectedCircleMaskIndex = hitIndex;
        // ドラッグ開始フラグはここではまだ立てない
        // ドラッグかクリックか判定するため mouseReleasedで判断してもよいが
        // ここではmouseDraggedで判断する
        // ひとまずフラグをリセット
        isCircleMaskDragging = false;
        isResizingCircleMask = false;
    }
    else
    {
        // どの円形マスクもヒットしなかった
        selectedCircleMaskIndex = -1;
    }
}


//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
    // 多角形マスクのドラッグ
    if (isDragging && selectedVertex >= 0)
    {
        const glm::vec2 newVertex = screenToWorld(x, y);
        maskVerticesParams[selectedVertex].set(newVertex);
        return;
    }

    // 円形マスクのドラッグ
    if (selectedCircleMaskIndex >= 0)
    {
        // SHIFT押下で半径変更
        bool      shiftPressed = (ofGetKeyPressed(OF_KEY_SHIFT));
        glm::vec2 centerScreen = worldToScreen(circleMasks[selectedCircleMaskIndex].center.get());
        glm::vec2 mousePos(x, y);

        if (!isCircleMaskDragging && !isResizingCircleMask)
        {
            // ドラッグ開始時点の判定
            if (shiftPressed)
            {
                // 半径変更モードへ
                isResizingCircleMask = true;
            }
            else
            {
                // 中心移動モードへ
                isCircleMaskDragging = true;
            }
        }

        if (isCircleMaskDragging && !shiftPressed)
        {
            // 中心移動
            const glm::vec2 newCenterWorld = screenToWorld(x, y);
            circleMasks[selectedCircleMaskIndex].center.set(newCenterWorld);
        }
        else if (isResizingCircleMask && shiftPressed)
        {
            // 半径変更
            const auto newRadiusScreen = glm::distance(mousePos, centerScreen);
            const auto newRadiusWorld  = newRadiusScreen / scale;
            circleMasks[selectedCircleMaskIndex].radius.set(newRadiusWorld);
        }
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
    // 多角形マスクのドラッグ完了
    isDragging     = false;
    selectedVertex = -1;

    // 円形マスクのドラッグ完了
    // ドラッグなしでクリックだけだった場合はactiveをトグルする
    if (selectedCircleMaskIndex >= 0 && !isCircleMaskDragging && !isResizingCircleMask)
    {
        // クリックのみ
        auto currentActive = circleMasks[selectedCircleMaskIndex].active.get();
        circleMasks[selectedCircleMaskIndex].active.set(!currentActive);
    }

    isCircleMaskDragging    = false;
    isResizingCircleMask    = false;
    selectedCircleMaskIndex = -1;
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

    updateMaskPath();
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
        const auto distance = length(point);
        if (distance < minDistance || distance > maxDistance)
        {
            continue;
        }

        // Rotate point using the rotation matrix
        glm::vec3 rotatedPoint = rotatePoint(point, rotationMatrix);

        // Apply inversion for ceiling mount
        if (floorMounted)
        {
            // Mirror the Y and Z axes
            rotatedPoint.y = -rotatedPoint.y;
            rotatedPoint.z = -rotatedPoint.z;
        }

        // Apply offsets
        rotatedPoint.x += xOffset;
        rotatedPoint.y += yOffset;
        rotatedPoint.z += zOffset;

        processedPoints.push_back(rotatedPoint);

        // Project the rotated point onto the grid plane (z = 0)
        glm::vec3 projectedPoint = rotatedPoint;
        projectedPoint.z         = 0;
        projectedPoints.push_back(projectedPoint);
    }
}

void ofApp::updateFbo()
{
    // 既存の処理(ポリゴンマスクによる削除) + 円形マスク追加

    // FBOのスワップ
    const ofFbo temp = fbo;
    fbo              = previousFbo;
    previousFbo      = temp;

    fbo.begin();
    ofEnableAlphaBlending();

    // 前回のFBOを減衰させて描画
    ofSetColor(255, 255, 255, 255 * fboDecayRate);
    previousFbo.draw(0, 0);

    // 減衰効果のための半透明黒
    ofSetColor(0, 0, 0, 255 * (1 - fboDecayRate));
    ofDrawRectangle(0, 0, fbo.getWidth(), fbo.getHeight());

    ofPushMatrix();
    ofTranslate(fbo.getWidth() / 2, fbo.getHeight() / 2);
    ofScale(scale, -scale); // 座標系統一

    // 点群描画
    ofSetColor(255, 255, 255, pointAlpha);
    for (const auto& point : projectedPoints)
    {
        float       adjustedPointSize = pointSize;
        const float distance          = glm::length(point);
        switch (pointSizeMode)
        {
        case 1:
            adjustedPointSize += pointSizeMultiplier * distance;
            break;
        case 2:
            adjustedPointSize += pointSizeMultiplier * distance * distance;
            break;
        default:
            break;
        }
        adjustedPointSize = ofClamp(adjustedPointSize, 0.1f, 500.0f);
        ofDrawCircle(point.x, point.y, adjustedPointSize);
    }

    // マスク描画 (ブレンドモードで点群削除)
    if (enableMask)
    {
        // 多角形マスク
        ofPushStyle();
        ofEnableBlendMode(OF_BLENDMODE_ALPHA);
        glEnable(GL_BLEND);
        glBlendFuncSeparate(GL_ZERO, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE);
        ofSetColor(255, 255, 255, 255);
        maskPath.draw();
        glDisable(GL_BLEND);
        ofPopStyle();

        // 円形マスク
        ofPushStyle();
        ofEnableBlendMode(OF_BLENDMODE_ALPHA);
        glEnable(GL_BLEND);
        glBlendFuncSeparate(GL_ZERO, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE);
        ofSetColor(255, 255, 255, 255);
        for (auto& cm : circleMasks)
        {
            if (!cm.active.get()) continue; // 非activeならスキップ
            ofDrawCircle(cm.center.get().x, cm.center.get().y, cm.radius.get());
        }
        glDisable(GL_BLEND);
        ofPopStyle();
    }

    ofPopMatrix();
    ofDisableAlphaBlending();
    fbo.end();
}

void ofApp::detectAndTrackBlobs()
{
    ofPixels pixels;
    fbo.readToPixels(pixels);

    const cv::Mat mat = ofxCv::toCv(pixels);
    cv::Mat       gray;
    cv::cvtColor(mat, gray, cv::COLOR_RGBA2GRAY);

    contourFinder.setThreshold(threshold);
    contourFinder.findContours(gray);

    // Get new and dead labels
    const std::vector<unsigned int>& newLabels  = contourFinder.getTracker().getNewLabels();
    const std::vector<unsigned int>& deadLabels = contourFinder.getTracker().getDeadLabels();

    // Send /blob/spawn messages for new blobs
    for (const auto& label : newLabels)
    {
        ofxOscMessage m;
        m.setAddress("/blob/spawn");
        m.addIntArg(label);
        sender.sendMessage(m, false);
    }

    // Send /blob/dead messages for dead blobs
    for (const auto& label : deadLabels)
    {
        ofxOscMessage m;
        m.setAddress("/blob/dead");
        m.addIntArg(label);
        sender.sendMessage(m, false);
    }
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
        center.y = (fbo.getHeight() / 2 - center.y) / scale; // Invert Y-axis

        if (floorMounted)
        {
            // Mirror Y-coordinate for inverted axes
            center.y = -center.y;
        }

        float ageInSeconds = contourFinder.getTracker().getAge(label) / ofGetFrameRate();

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
            const float angle    = glm::degrees(atan2(center.y, center.x));
            m.addIntArg(label);
            m.addFloatArg(angle);    // Angle in degrees
            m.addFloatArg(distance); // Distance in mm
            m.addFloatArg(ageInSeconds);
        }
        sender.sendMessage(m, false); // Non-blocking send
    }
}

void ofApp::checkResetFbo()
{
    if (const auto currentTime = ofGetElapsedTimef(); currentTime - lastOscReceiveTime > resetTimeout)
    {
        shouldResetFbo = true;
        resetFboAndPcl();
    }
}

void ofApp::resetFboAndPcl()
{
    // Clear both FBOs
    fbo.begin();
    ofClear(0, 0, 0, 0);
    fbo.end();
    previousFbo.begin();
    ofClear(0, 0, 0, 0);
    previousFbo.end();

    // Clear point clouds
    processedPoints.clear();
    projectedPoints.clear();
    pointMesh.clear();
    projectedMesh.clear();
}


void ofApp::initializeMask()
{
    ofxXmlSettings settings;
    bool           hasFile = settings.load("settings.xml");

    maskVerticesParams.clear();
    circleMasks.clear(); // 円マスククリア

    maskParams.setName("Mask Vertices");
    circleMaskParams.setName("Circle Masks");

    int numVertices = 6; // デフォルト6頂点
    if (hasFile)
    {
        settings.pushTag("Settings");
        if (settings.tagExists("Mask_Vertices"))
        {
            settings.pushTag("Mask_Vertices");
            auto count = 0;
            while (true)
            {
                std::string vertexName = "Vertex_" + ofToString(count);
                if (!settings.tagExists(vertexName))
                {
                    break;
                }
                count++;
            }
            settings.popTag(); // pop Mask_Vertices

            if (count > 0)
            {
                // ファイルに頂点が記載されていれば、その数を使用
                numVertices = count;
            }
        }
        settings.popTag(); // pop Settings
    }

    // numVertices個の頂点パラメータ生成
    for (auto i = 0; i < numVertices; i++)
    {
        // 初期値は六角形状に配置したい場合は元のコードと同様に計算する
        float     radius = 2000;
        float     angle  = i * TWO_PI / 6;
        glm::vec2 defaultVertex(radius * cos(angle), radius * sin(angle));

        ofParameter<glm::vec2> vertexParam;
        vertexParam.set("Vertex " + ofToString(i), defaultVertex, glm::vec2(-10000, -10000), glm::vec2(10000, 10000));
        maskParams.add(vertexParam);
        maskVerticesParams.push_back(vertexParam);
    }

    // 円マスク数を判定（なければ0個）
    auto numCircleMasks = 0;
    if (hasFile)
    {
        settings.pushTag("Settings");
        if (settings.tagExists("Circle_Masks"))
        {
            settings.pushTag("Circle_Masks");
            while (true)
            {
                std::string maskTag = "CircleMask_" + ofToString(numCircleMasks);
                if (!settings.tagExists(maskTag))
                {
                    break;
                }
                numCircleMasks++;
            }
            settings.popTag(); // pop Circle_Masks
        }
        settings.popTag(); // pop Settings
    }

    // numCircleMasks個の円マスクパラメータ生成 (0なら何もしない)
    for (auto i = 0; i < numCircleMasks; i++)
    {
        CircleMask cm;
        cm.group.setName("CircleMask_" + ofToString(i));
        cm.center.set("Center", glm::vec2(0, 0), glm::vec2(-10000, -10000), glm::vec2(10000, 10000));
        cm.radius.set("Radius", 200.0f, 1.0f, 5000.0f);
        cm.active.set("Active", true);

        cm.group.add(cm.center);
        cm.group.add(cm.radius);
        cm.group.add(cm.active);

        circleMaskParams.add(cm.group);
        circleMasks.push_back(cm);
    }

    updateMaskPath();
}

void ofApp::updateMaskPath()
{
    maskPath.clear();
    maskPath.setFilled(true);
    maskPath.setColor(ofColor(255));

    // Build the path from parameters
    maskPath.moveTo(maskVerticesParams[0].get());
    for (auto i = 1; i < maskVerticesParams.size(); i++)
    {
        maskPath.lineTo(maskVerticesParams[i].get());
    }
    maskPath.close();
}


void ofApp::drawMask() const
{
    // マスクの輪郭を半透明の色で描画（スケール変換内）
    ofPushMatrix();
    ofTranslate(ofGetWidth() / 2, ofGetHeight() / 2);
    ofScale(scale, -scale); // スケールと座標系を調整

    ofPushStyle();
    ofSetColor(0, 0, 255, 100); // 半透明の青色
    maskPath.draw();
    ofPopStyle();

    ofPopMatrix();

    // ハンドルをスクリーン座標で描画（スケール変換外）
    ofPushStyle();
    ofNoFill();            // 塗りつぶしなし
    ofSetColor(255, 0, 0); // 赤色
    ofSetLineWidth(2);     // 線の太さを調整

    for (const auto& vertexParam : maskVerticesParams)
    {
        glm::vec2 screenPos = worldToScreen(vertexParam.get());
        ofDrawCircle(screenPos, 15); // ハンドルの半径を15に調整
    }

    ofPopStyle();
}

glm::vec2 ofApp::screenToWorld(float x, float y) const
{
    glm::vec2 worldPos;
    worldPos.x = (x - ofGetWidth() / 2) / scale;
    worldPos.y = (ofGetHeight() / 2 - y) / scale; // Y軸を反転
    return worldPos;
}

glm::vec2 ofApp::worldToScreen(const glm::vec2& worldPos) const
{
    glm::vec2 screenPos;
    screenPos.x = ofGetWidth() / 2 + worldPos.x * scale;
    screenPos.y = ofGetHeight() / 2 - worldPos.y * scale; // Y軸を反転
    return screenPos;
}

void ofApp::updateMaskPath(ofAbstractParameter&)
{
    updateMaskPath();
}

// 円形マスク追加処理
void ofApp::addCircleMask(const glm::vec2& centerWorld)
{
    CircleMask cm;
    cm.group.setName("CircleMask " + ofToString(circleMasks.size()));
    cm.center.set("Center", centerWorld, glm::vec2(-10000, -10000), glm::vec2(10000, 10000));
    cm.radius.set("Radius", 200.0f, 1.0f, 5000.0f);
    cm.active.set("Active", true);

    cm.group.add(cm.center);
    cm.group.add(cm.radius);
    cm.group.add(cm.active);

    circleMaskParams.add(cm.group);
    circleMasks.push_back(cm);
}

int ofApp::hitTestCircleMaskScreen(int x, int y) const
{
    // スクリーン座標をマスク中心に変換し，半径以内かどうか
    for (auto i = static_cast<int>(circleMasks.size()) - 1; i >= 0; i--)
    {
        glm::vec2  centerScreen = worldToScreen(circleMasks[i].center.get());
        const auto radiusScreen = circleMasks[i].radius.get() * scale; // 半径もスケール適用
        if (const auto dist = glm::distance(glm::vec2(x, y), centerScreen); dist < radiusScreen)
        {
            return i;
        }
    }
    return -1;
}

// 円形マスクの表示
void ofApp::drawCircleMasks() const
{
    ofPushStyle();
    // スクリーン上に中心と半径を描画
    for (auto i = 0; i < static_cast<int>(circleMasks.size()); i++)
    {
        glm::vec2  centerScreen = worldToScreen(circleMasks[i].center.get());
        const auto radiusScreen = circleMasks[i].radius.get() * scale;

        // activeなものは青、非activeはグレー
        ofColor c = circleMasks[i].active.get() ? ofColor(0, 0, 255, 100) : ofColor(100, 100, 100, 100);
        ofSetColor(c);
        ofDrawCircle(centerScreen, radiusScreen);

        // 中心点ハンドル
        ofSetColor(255, 0, 0);
        ofDrawCircle(centerScreen, 5);
    }
    ofPopStyle();
}
