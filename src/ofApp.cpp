#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup(){

	ofBackground(0);
	ofSetWindowShape(colW, colH); // color stream w x h

	kinect.open();
	kinect.initColorSource();
	kinect.initDepthSource();
	kinect.initBodySource();

	if (kinect.getSensor()->get_CoordinateMapper(&coordinateMapper) < 0) {
		ofLogError("setup") << "couldn't aquire coordinate mapper!";
	} else {
		ofLogNotice("setup") << "aquired coordinate mapper";
	}

	bStreams = false;
	kColImg.allocate(colW,colH,OF_IMAGE_COLOR);

	ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::update(){

	kinect.update();

	ofPixels& colorPix = kinect.getColorSource()->getPixels();

	if (!colorPix.size()) { // has data?
		bStreams = false;
		ofLogError("update") << "not reading streams from kinect yet";
		return;
	}
	else {
		kColImg.setFromPixels(colorPix);
		bStreams = true;
	}

	// get num tracked bodies
	numTracked = 0;
	for (auto& body : kinect.getBodySource()->getBodies()) {
		if (body.tracked) {
			numTracked++;
		}
	}
	bodyIdx = -1;
	if (numTracked > 0) {

		// get centermost tracked body
		bodyIdx = getCentralBodyIdx();

		if (bodyIdx >= 0) { // double check but should always be true since we have tracked bodies
			
			// get ptr to the joints
			auto& bodies = kinect.getBodySource()->getBodies();
			auto& joints = bodies[bodyIdx].joints;

			// convert world coords to screen coords
			jts2d.clear();
			for (auto& jt : joints) {
				ofVec2f& p = jts2d[jt.second.getType()] = ofVec2f();

				TrackingState state = jt.second.getTrackingState();
				if (state == TrackingState_NotTracked) continue; // skip joint

				p.set(jt.second.getProjected(coordinateMapper)); // proj world to color cam plz!
			}
		}
	}

}

//--------------------------------------------------------------
void ofApp::draw(){

	ofSetColor(255);
	kColImg.draw(0, 0, 1920, 1080);

	if (numTracked > 0 && bodyIdx >= 0) {

		//kinect.getBodySource()->drawProjected(0, 0, 1920, 1080);

		// draw joints
		auto& body = kinect.getBodySource()->getBodies()[bodyIdx];
		for (auto& jt : body.joints) {

			TrackingState state = jt.second.getTrackingState();
			if (state == TrackingState_NotTracked) continue; // skip
			int rad = state == TrackingState_Inferred ? 2 : 8;
			ofVec2f& pos = jts2d[jt.second.getType()];
			ofSetColor(0, 255, 0);
			ofDrawCircle(pos, rad);			
		}

		// draw hand states
		ofColor left; bool trackLeft = true;
		switch (body.leftHandState){
			case HandState_Unknown: case HandState_NotTracked:
				trackLeft = false;
				break;
			case HandState_Open:
				left = ofColor(0, 255, 0, 80);
				break;
			case HandState_Closed:
				left = ofColor(255, 255, 0, 80);
				break;
			case HandState_Lasso:
				left = ofColor(0, 255, 255, 80);
				break;
		}
		if (trackLeft) {
			ofEnableAlphaBlending();
			ofSetColor(left);
			ofCircle(jts2d[JointType_HandLeft], 50);
			ofDisableAlphaBlending();
		}

		ofColor right; bool trackRight = true;
		switch (body.rightHandState) {
		case HandState_Unknown: case HandState_NotTracked:
			trackRight = false;
			break;
		case HandState_Open:
			right = ofColor(0, 255, 0, 80);
			break;
		case HandState_Closed:
			right = ofColor(255, 255, 0, 80);
			break;
		case HandState_Lasso:
			right = ofColor(0, 255, 255, 80);
			break;
		}
		if (trackRight) {
			ofEnableAlphaBlending();
			ofSetColor(right);
			ofCircle(jts2d[JointType_HandRight], 50);
			ofDisableAlphaBlending();
		}


	}

	stringstream infoss;
	infoss << "num tracked: " << ofToString(numTracked) << endl
		<< "body idx: " << ofToString(bodyIdx) << endl
		<< "fps: " << ofToString(ofGetFrameRate()) << endl;

	ofDrawBitmapStringHighlight(infoss.str(), 10, 10);
}

//--------------------------------------------------------------
int ofApp::getCentralBodyIdx() {

	auto & bodies = kinect.getBodySource()->getBodies();

	int closestIdx = -1;
	float closestDist = 99999.f;
	for (int i = 0; i < bodies.size(); i++) {
		if (!bodies[i].tracked) continue; // skip untracked body
		float dist = abs(bodies[i].joints.at(JointType_SpineBase).getPosition().x);
		if (dist < closestDist) {
			closestIdx = i;
			closestDist = dist;
		}
	}
	return closestIdx;
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
