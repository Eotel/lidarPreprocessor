#pragma once

#include "ofMain.h"


#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxMeshWarp.h"
#include "ofxMeshWarpManagedController.h"
#include "ofxOsc.h"

struct CircleMask
{
    ofParameterGroup       group;
    ofParameter<glm::vec2> center;
    ofParameter<float>     radius;
    ofParameter<bool>      active;
};

/**
 * @class ofApp
 * @brief The core application class used in OpenFrameworks to manage and render the main application logic.
 *
 * This class is the entry point for handling application setup, updates, and rendering processes. It integrates
 * with the OpenFrameworks framework to provide functionality for graphical applications, handling input events,
 * managing the rendering loop, and interfacing with other OpenFrameworks features.
 */
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

    ofParameterGroup   params;
    ofParameter<float> yawOffset;
    ofParameter<float> rollOffset;
    ofParameter<float> pitchOffset;
    ofParameter<float> xOffset;
    ofParameter<float> yOffset;
    ofParameter<float> zOffset;
    ofParameter<float> minDistance;
    ofParameter<float> maxDistance;
    ofParameter<float> pointSize;
    ofParameter<int>   pointAlpha;
    ofParameter<int>   pointSizeMode;
    ofParameter<float> pointSizeMultiplier;
    ofParameter<bool>  floorMounted{true}; // false: ceiling mounted

    ofParameterGroup    oscParams;
    ofParameter<string> oscSendIP;
    ofParameter<int>    oscSendPort;
    ofParameter<int>    oscReceivePort;
    ofParameter<bool>   sendCartesian;

    ofParameterGroup   trackerParams;
    ofParameter<float> minArea;
    ofParameter<float> maxArea;
    ofParameter<float> threshold;
    ofParameter<bool>  findHoles;
    ofParameter<int>   persistence;
    ofParameter<float> maximumDistance;

    ofParameterGroup   viewParams;
    ofParameter<bool>  showPCL;
    ofParameter<bool>  showFBO;
    ofParameter<bool>  showTracker;
    ofParameter<bool>  showGrid;
    ofParameter<bool>  showFBOGrid;
    ofParameter<float> scale;
    ofParameter<float> gridSpacing;

    ofxMeshWarp       meshWarp;
    ofParameter<bool> showMeshWarp;
    ofParameter<int>  meshWarpCols;
    ofParameter<int>  meshWarpRows;

    ofPath            maskPath;
    vector<glm::vec2> maskVertices;
    int               selectedVertex{-1};
    bool              isDragging{false};


    ofParameterGroup        circleMaskParams;                // 全円形マスクをまとめるグループ
    std::vector<CircleMask> circleMasks;                     // 複数の円形マスク
    int                     selectedCircleMaskIndex = -1;    // 選択中の円形マスク
    bool                    isCircleMaskDragging    = false; // ドラッグ中か
    bool                    isResizingCircleMask    = false; // 半径変更中かどうか

    void addCircleMask(const glm::vec2& centerWorld);
    int  hitTestCircleMaskScreen(int x, int y) const; // マウス座標でどのマスクが当たるか判定
    void drawCircleMasks() const;

    void      initializeMask();
    void      drawMask() const;
    void      updateMaskPath();
    void      updateMaskPath(ofAbstractParameter& parameter);
    glm::vec2 screenToWorld(float x, float y) const;
    glm::vec2 worldToScreen(const glm::vec2& worldPos) const;

    ofParameter<bool>                   enableMask;
    ofParameterGroup                    maskParams;
    std::vector<ofParameter<glm::vec2>> maskVerticesParams;

    ofxButton loadButton;
    ofxButton saveButton;

    // Data processing
    std::vector<glm::vec3> processedPoints;
    std::vector<glm::vec3> projectedPoints;
    void                   processPoints(const std::vector<glm::vec3>& points);

    // Rendering
    ofFbo fbo;
    ofFbo previousFbo;
    void  updateFbo();

    // Blob detection and tracking
    ofxCv::ContourFinder contourFinder;
    void                 detectAndTrackBlobs();

    void loadSettings();
    void saveSettings();
    void oscSetup(ofAbstractParameter& e);
    void trackerSetup(ofAbstractParameter& e);
    void updateRotationMatrix(ofAbstractParameter& e);

    ofxOscReceiver receiver;
    ofxOscSender   sender;
    void           sendTrackedBlobs();

    // Utility functions
    glm::mat4        rotationMatrix;
    static glm::vec3 rotatePoint(const glm::vec3& point, const glm::mat4& rotationMatrix);

    // Debug parameters
    ofParameterGroup    debugParams;
    ofParameter<int>    fps;
    ofParameter<string> blobInfo;

    // GUI visibility
    bool showGUI = true;

    ofEasyCam easyCam;

    ofVboMesh pointMesh;
    ofVboMesh projectedMesh;

    int ofWidth{0};
    int ofHeight{0};

    ofParameterGroup   resetParams;
    ofParameter<float> resetTimeout;
    ofParameter<float> fboDecayRate;

    // Tracking last OSC receive time and FBO reset
    float lastOscReceiveTime{0};
    bool  shouldResetFbo{false};

    // Check if FBO should be reset
    void checkResetFbo();

    // To reset FBO and PCL
    void resetFboAndPcl();

    void initializeFbo() const;
};
