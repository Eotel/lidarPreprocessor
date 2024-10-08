#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxOsc.h"

class ofApp final : public ofBaseApp
{
public:
    void setup() override;
    void update() override;
    void drawGrid() const;
    void drawPCL();
    void drawFBO() const;
    void drawFBOGrid() const;
    void drawFBOGridLabels() const;
    void draw() override;
    void exit() override;

    void keyPressed(int key) override;
    void keyReleased(int key) override;
    void mouseMoved(int x, int y) override;
    void mouseDragged(int x, int y, int button) override;
    void mousePressed(int x, int y, int button) override;
    void mouseReleased(int x, int y, int button) override;
    void mouseScrolled(int x, int y, float scrollX, float scrollY) override;
    void mouseEntered(int x, int y) override;
    void mouseExited(int x, int y) override;
    void windowResized(int w, int h) override;
    void dragEvent(ofDragInfo dragInfo) override;
    void gotMessage(ofMessage msg) override;

private:
    ofxPanel guiPanel;

    ofParameterGroup params;
    ofParameter<float> yawOffset;
    ofParameter<float> rollOffset;
    ofParameter<float> pitchOffset;
    ofParameter<float> xOffset;
    ofParameter<float> yOffset;
    ofParameter<float> zOffset;
    ofParameter<float> minDistance;
    ofParameter<float> maxDistance;
    ofParameter<float> pointSize;
    ofParameter<int> pointAlpha;

    ofParameter<int> pointSizeMode;
    ofParameter<float> pointSizeMultiplier;

    ofParameterGroup oscParams;
    ofParameter<string> oscSendIP;
    ofParameter<int> oscSendPort;
    ofParameter<int> oscReceivePort;
    ofParameter<bool> sendCartesian;

    ofParameterGroup trackerParams;
    ofParameter<float> minArea;
    ofParameter<float> maxArea;
    ofParameter<float> threshold;
    ofParameter<bool> findHoles;
    ofParameter<int> persistence;
    ofParameter<float> maximumDistance;

    ofParameterGroup viewParams;
    ofParameter<bool> showPCL;
    ofParameter<bool> showFBO;
    ofParameter<bool> showTracker;
    ofParameter<bool> showGrid;
    ofParameter<bool> showFBOGrid;
    ofParameter<float> scale;
    ofParameter<float> gridSpacing;

    ofxButton loadButton;
    ofxButton saveButton;

    // Data processing
    std::vector<glm::vec3> processedPoints;
    std::vector<glm::vec3> projectedPoints;
    void processPoints(const std::vector<glm::vec3>& points);

    // Rendering
    ofFbo fbo;
    void updateFbo() const;

    // Blob detection and tracking
    ofxCv::ContourFinder contourFinder;
    void detectAndTrackBlobs();

    void loadSettings();
    void saveSettings();
    void oscSetup(ofAbstractParameter& e);
    void trackerSetup(ofAbstractParameter& e);
    void updateRotationMatrix(ofAbstractParameter& e);

    ofxOscReceiver receiver;
    ofxOscSender sender;
    void sendTrackedBlobs();
    void sendTrackedBlobs(int lidarID);

    // Utility functions
    glm::mat4 rotationMatrix;
    static glm::vec3 rotatePoint(const glm::vec3& point, const glm::mat4& rotationMatrix);

    // Debug parameters
    ofParameterGroup debugParams;
    ofParameter<int> fps;
    ofParameter<string> blobInfo;

    // GUI visibility
    bool showGUI = true;

    ofEasyCam easyCam;

    ofVboMesh pointMesh;
    ofVboMesh projectedMesh;

    int ofWidth{0};
    int ofHeight{0};
};
