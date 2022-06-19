import QtQuick 2.14
import QtQuick.Controls 2.14
import "../controls"

Item {

    QtObject {
        id: trackingParams
        property variant points:        []
        property variant line:          []
        property variant clutters:      []
        property variant measurements:  []
    }

    QtObject {
        id: stateMachine
        readonly property int draw_line :  1
        readonly property int clutter   :  2
        property int state: draw_line
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

        ButtonLabel {
            id: btnClear
            btnText: "erase"
            btnIconSource: "../../ui/images/svg_images/eraser.svg"
            onClicked: {
                trackingParams.points       = []
                trackingParams.line         = []
                trackingParams.clutters     = []
                trackingParams.measurements = []
                canvas.requestPaint()
            }
        }

        ButtonLabel {
            id: btnDrawLine
            btnText: "draw"
            btnIconSource: "../../ui/images/svg_images/draw_line.svg"
            onClicked: {
                if(stateMachine.state == stateMachine.clutter) {
                    stateMachine.state = stateMachine.draw_line
                }
            }
        }

        ButtonLabel {
            id: btnClutter
            btnText: "clutter"
            btnIconSource: "../../ui/images/svg_images/clutter.svg"
            onClicked: {
                if(stateMachine.state == stateMachine.draw_line) {
                    stateMachine.state = stateMachine.clutter
                }
            }
        }

        ButtonLabel {
            id: btnMeasurements
            btnText: "measure"
            btnIconSource: "../../ui/images/svg_images/measurement.svg"
            onClicked: {
                trackingParams.measurements = []
                for(var i = 0; i < trackingParams.points.length; i++) {
                    var x = trackingParams.points[i][0] + (Math.random() - 0.5) * 30
                    var y = trackingParams.points[i][1] + (Math.random() - 0.5) * 30
                    trackingParams.measurements.push( [ parseInt(x), parseInt(y) ] )
                }
                canvas.requestPaint()
            }
        }

        ButtonLabel {
            id: btnRun
            btnText: "Run"
            btnIconSource: "../../ui/images/svg_images/run.svg"
            onClicked: {
                // console.log( trackingParams.measurements )
                var xs = [];
                var ys = [];
                for(var i = 0; i < trackingParams.measurements.length; i++) {
                    xs.push( trackingParams.measurements[i][0] )
                    ys.push( trackingParams.measurements[i][1] )
                }
                backend.getMeasurements( xs, ys )
            }
        }


    }

    Canvas {
        id: canvas
        anchors {
            left:   parent.left
            right:  parent.right
            top:    colorTools.bottom
            bottom: parent.bottom
            margins: 8
        }
        property real lastX
        property real lastY
        property color color: colorTools.paintColor

        onPaint: {
            // console.log("onPaint")
            var ctx = getContext('2d')

            ctx.fillStyle = Qt.rgba(0.5, 0.5, 0.5, 1);
            ctx.fillRect(0, 0, width, height);

            var x = 0
            var y = 0
            for(var i = 0; i < trackingParams.line.length - 1; i++) {
                ctx.lineWidth = 3
                ctx.strokeStyle = canvas.color
                ctx.beginPath()
                x = trackingParams.line[i][0]
                y = trackingParams.line[i][1]
                ctx.moveTo(x, y)
                x = trackingParams.line[i + 1][0]
                y = trackingParams.line[i + 1][1]
                ctx.lineTo(x, y)
                ctx.stroke()
            }

            for(i = 0; i < trackingParams.points.length; i++) {
                ctx.beginPath();
                ctx.fillStyle = colorTools.paintColor
                x = trackingParams.points[i][0]
                y = trackingParams.points[i][1]
                ctx.arc(x, y, 5, 0, Math.PI * 2, false);
                ctx.fill();
            }

            for(i = 0; i < trackingParams.clutters.length; i++) {
                ctx.beginPath();
                ctx.fillStyle = "white"
                x = trackingParams.clutters[i][0]
                y = trackingParams.clutters[i][1]
                ctx.arc(x, y, 3, 0, Math.PI * 2, false);
                ctx.fill();
            }

            for(i = 0; i < trackingParams.measurements.length; i++) {
                ctx.beginPath();
                ctx.fillStyle = "white"
                x = trackingParams.measurements[i][0]
                y = trackingParams.measurements[i][1]
                ctx.arc(x, y, 3, 0, Math.PI * 2, false);
                ctx.fill();
            }


        }

        MouseArea {
            id: area
            anchors.fill: parent
            onPressed: {
                if(stateMachine.state == stateMachine.draw_line) {
                    trackingParams.points = []
                    trackingParams.points.push( [ parseInt(mouseX), parseInt(mouseY) ] )
                    trackingParams.line = []
                } else if(stateMachine.state == stateMachine.clutter) {
                    var x = parseInt( mouseX )
                    var y = parseInt( mouseY )
                    for(var i = 0; i < 5; i++) {
                        var currX = x + (Math.random() - 0.5) * 100
                        var currY = y + (Math.random() - 0.5) * 100
                        trackingParams.clutters.push( [ currX, currY ] )
                    }
                    canvas.requestPaint()
                }
            }
            onReleased: {
                // trackingParams.path.push( trackingParams.currPoints )
                // trackingParams.currPoints = []
            }
            onPositionChanged: {
                if(stateMachine.state == stateMachine.draw_line) {
                    trackingParams.line.push( [parseInt(mouseX), parseInt(mouseY)] )
                    var lastX = trackingParams.points[trackingParams.points.length - 1][0]
                    var lastY = trackingParams.points[trackingParams.points.length - 1][1]
                    if( Math.hypot(lastX - mouseX, lastY - mouseY) > 40) {
                        trackingParams.points.push( [ parseInt(mouseX), parseInt(mouseY) ] )
                    }
                }
                canvas.requestPaint()
            }
        }
    }


}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:800}
}
##^##*/