import QtQuick 2.12
import QtQuick.Controls 2.12

Item {

    QtObject {
        id: trackingParams
        property bool shouldClear : true
        property variant currPoints: []
        property variant path: []
        property string typeOfDraw: "GroundTruth"
        property variant xToDraw: []
        property variant yToDraw: []
        property real lastX
        property real lastY
        property bool drawCircle: false
    }

    Row {

        id: colorTools
        height: btnClear.height
        width: parent.width - 20
        spacing: 10
        anchors {
            horizontalCenter: parent.horizontalCenter
            top: parent.top
            topMargin: 8
        }
        property color paintColor: "#33B5E5"

        Button {
            id: btnClear
            text: "Clear Screen"
            onClicked: {
                trackingParams.shouldClear = true
                canvas.requestPaint()
            }
        }

        Button {
            id: btnShowPath
            text: "Show Path"
            onClicked: {
                // console.log( trackingParams.path )
                // backend.receiveFromQml("Mohsen");
                // console.log( trackingParams.pointToDraw.length )
            }
        }

    }

    Canvas {
        id: canvas
        anchors {
            left: parent.left
            right: parent.right
            top: colorTools.bottom
            bottom: parent.bottom
            margins: 8
        }
        property real lastX
        property real lastY
        property color color: colorTools.paintColor

        onPaint: {
            // console.log("onPaint")
            var ctx = getContext('2d')
            if(trackingParams.shouldClear == true) {
                ctx.fillStyle = Qt.rgba(0.5, 0.5, 0.5, 1);
                ctx.fillRect(0, 0, width, height);
                trackingParams.shouldClear = false
            }

            if(trackingParams.drawCircle == true) {
                ctx.beginPath();
                ctx.fillStyle = colorTools.paintColor
                ctx.moveTo(trackingParams.lastX, trackingParams.lastY);
                ctx.arc(trackingParams.lastX, trackingParams.lastY, 5, 0, Math.PI * 2, false);
                ctx.lineTo(trackingParams.lastX, trackingParams.lastY);
                ctx.fill();
                trackingParams.drawCircle = false
            }

            if(trackingParams.xToDraw.length != 0) {
                var circleColor
                var circleRadius
                if(trackingParams.typeOfDraw == "GroundTruth") {
                    circleColor = "white"
                    circleRadius = 3
                } else {
                    circleColor = "blue"
                    circleRadius = 2
                }

                for(var i = 0; i < trackingParams.xToDraw.length; i++) {
                    ctx.beginPath();
                    ctx.fillStyle = circleColor
                    var x = trackingParams.xToDraw[i]
                    var y = canvas.height - trackingParams.yToDraw[i]
                    // console.log(x, y)
                    if(trackingParams.typeOfDraw == "GroundTruth" && i > 0) {
                        ctx.lineWidth = 2
                        ctx.strokeStyle = circleColor
                        var preX = trackingParams.xToDraw[i - 1]
                        var preY = canvas.height - trackingParams.yToDraw[i - 1]
                        ctx.moveTo(preX, preY);
                        ctx.lineTo(x, y);
                        ctx.stroke()
                    }
                    ctx.arc(x, y, circleRadius, 0, Math.PI * 2, false);
                    ctx.fill();
                }
                trackingParams.xToDraw = []
                trackingParams.yToDraw = []
            }

            ctx.lineWidth = 3
            ctx.strokeStyle = canvas.color
            ctx.beginPath()
            ctx.moveTo(lastX, lastY)
            lastX = area.mouseX
            lastY = area.mouseY
            ctx.lineTo(lastX, lastY)
            ctx.stroke()
        }

        MouseArea {
            id: area
            anchors.fill: parent
            onPressed: {
                canvas.lastX = mouseX
                canvas.lastY = mouseY
                trackingParams.currPoints.push( [ parseInt(mouseX), parseInt(mouseY) ] )
                trackingParams.drawCircle = true
                trackingParams.lastX = mouseX
                trackingParams.lastY = mouseY
            }
            onReleased: {
                trackingParams.path.push( trackingParams.currPoints )
                trackingParams.currPoints = []
            }
            onPositionChanged: {
                if( Math.hypot(trackingParams.lastX - mouseX, trackingParams.lastY - mouseY) > 40) {
                    trackingParams.currPoints.push( [ parseInt(mouseX), parseInt(mouseY) ] )
                    trackingParams.lastX = mouseX
                    trackingParams.lastY = mouseY
                    trackingParams.drawCircle = true
                }
                canvas.requestPaint()
            }
        }
    }

    Connections {
        target: backend

        function onRecvData(text) {
            console.log("onRecvData " + text)
        }
        function onAddData(typeOfDraw, x, y) {
            // console.log("Received Data")
            trackingParams.typeOfDraw = typeOfDraw
            trackingParams.xToDraw = x
            trackingParams.yToDraw = y
            canvas.requestPaint()
        }
    }


}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:800}
}
##^##*/
