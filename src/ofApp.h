#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxOsc.h"
// #include "ofxSyphon.h"
#include "ofxXmlSettings.h"

class ofApp final : public ofBaseApp
{
public:
    void setup() override;
    void update() override;
    void drawGrid() const;
    void drawPCL() const;
    void drawFBO();
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
    ofParameter<float> minDistance;
    ofParameter<float> maxDistance;

    ofParameter<float> pointSize;
    ofParameter<int> pointAlpha;

    ofParameterGroup oscParams;
    ofParameter<string> oscSendIP;
    ofParameter<int> oscSendPort;
    ofParameter<int> oscReceivePort;

    ofParameterGroup trackerParams;
    ofParameter<float> minArea;
    ofParameter<float> maxArea;
    ofParameter<int> threshold;
    ofParameter<bool> findHoles;
    ofParameter<int> persistence;
    ofParameter<float> maximumDistance;

    ofParameterGroup viewParams;
    ofParameter<bool> showPCL;
    ofParameter<bool> showFBO;
    ofParameter<bool> showTracker;
    ofParameter<bool> showGrid;
    ofParameter<float> scale;
    ofParameter<float> gridSpacing;

    ofxButton loadButton;
    ofxButton saveButton;

    // Data processing
    std::vector<glm::vec3> processedPoints;
    void processPoints(const std::vector<glm::vec3>& points);

    // Rendering
    ofFbo fbo;
    void updateFbo() const;

    // Blob detection and tracking
    ofxCv::ContourFinder contourFinder;
    ofxCv::RectTracker tracker;
    void detectAndTrackBlobs();

    void loadSettings();
    void saveSettings();
    void oscSetup(ofAbstractParameter& e);
    void trackerSetup(ofAbstractParameter& e);

    ofxOscReceiver receiver;
    ofxOscSender sender;
    void sendTrackedBlobs();

    static ofVec3f rotatePoint(const ofVec3f& point, float roll, float pitch, float yaw);

    // Syphon server
    // ofxSyphonServer syphonServer;

    // Debug parameters
    ofParameterGroup debugParams;
    ofParameter<float> fps;
    ofParameter<string> blobInfo;

    // GUI visibility
    bool showGUI = true;

    // ofEasyCam for camera control
    ofEasyCam easyCam;

    // ofVboMesh for efficient point cloud rendering
    ofVboMesh pointMesh;
};
