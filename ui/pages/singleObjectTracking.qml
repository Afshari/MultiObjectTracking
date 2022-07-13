import QtQuick 2.12
import QtQml 2.12
import QtQuick.Controls 2.12
import "../controls"


Item {

    QtObject {
        id: trackingParams
        property variant points:                    []
        property variant line:                      []
        property variant clutters:                  []
        property variant measurements:              []
        property variant all_measure:               []
        property variant nearest_neighbor:          []
        property variant pda:                       []
        property variant gaussian_sum:              []
        property bool show_nearest_neighbor:        true
        property bool show_pda:                     true
        property bool show_gaussian_sum:            true
        property bool show_measurement:             true
        property bool show_all_meas:                true
        property bool show_truth:                   true

    }

    function drawPoints(ctx, items, color) {

        var x = 0;
        var y = 0;
        for(var i = 0; i < items.length; i++) {
            ctx.beginPath();
            ctx.strokeStyle = color
            x = items[i][0]
            y = items[i][1]
            ctx.arc(x, y, 5, 0, Math.PI * 2, false);
            ctx.stroke();
        }
    }

    function drawLegends(ctx, text_start_y, color, txt, TEXT_START_X, text_start_y, SYM_START_X) {

        ctx.beginPath();
        ctx.fillStyle = color;
        ctx.fillText(txt, TEXT_START_X, text_start_y);
        ctx.arc(SYM_START_X, text_start_y - 5, 5, 0, Math.PI * 2, false);
        ctx.fill()
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
        property color truth:               "#33B5E5"
        property color nearest_neighbor:    "#FF0000"
        property color pda:                 "#00FF00"
        property color gaussian_sum:        "#0000FF"
        property color measurement:         "white"

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
                console.log("Button Measure Clicked");
                trackingParams.measurements = []
                for(var i = 0; i < trackingParams.points.length; i++) {
                    var x = trackingParams.points[i][0] + (Math.random() - 0.5) * 30
                    var y = trackingParams.points[i][1] + (Math.random() - 0.5) * 30
                    trackingParams.measurements.push( [ parseInt(x), parseInt(y) ] )
                }
                canvas.requestPaint();
                backend.qmlCommand();
            }
        }

        ButtonLabel {
            id: btnRun
            btnText: "Run"
            btnIconSource: "../../ui/images/svg_images/run.svg"
            onClicked: {
                // console.log( trackingParams.measurements )
//                var xs = [];
//                var ys = [];
//                for(var i = 0; i < trackingParams.measurements.length; i++) {
//                    xs.push( trackingParams.measurements[i][0] )
//                    ys.push( trackingParams.measurements[i][1] )
//                }
                // backend.getMeasurements( xs, ys )
                // backend.receiveFromQml("OK");
                backend.qmlCommand("single");
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "NN"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                trackingParams.show_nearest_neighbor = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "PDA"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                trackingParams.show_pda = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "GS"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                trackingParams.show_gaussian_sum = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "Measure"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                trackingParams.show_measurement = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "All Meas"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                trackingParams.show_all_meas = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "Truth"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                trackingParams.show_truth = checked
                canvas.requestPaint()
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
        property color color: colorTools.truth

        onPaint: {
            // console.log("onPaint")
            var ctx = getContext('2d')

            ctx.fillStyle = Qt.rgba(0.5, 0.5, 0.5, 1);
            ctx.fillRect(0, 0, width, height);

            var x = 0
            var y = 0

            if(trackingParams.show_truth == true) {
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
            }

            if(trackingParams.show_nearest_neighbor == true)    drawPoints(ctx, trackingParams.nearest_neighbor, colorTools.nearest_neighbor)
            if(trackingParams.show_pda == true)                 drawPoints(ctx, trackingParams.pda, colorTools.pda)
            if(trackingParams.show_gaussian_sum == true)        drawPoints(ctx, trackingParams.gaussian_sum, colorTools.gaussian_sum)


            if(trackingParams.show_measurement == true) {

                if(trackingParams.show_all_meas == true) {
                    for(i = 0; i < trackingParams.all_measure.length; i++) {
                        ctx.beginPath();
                        ctx.fillStyle = colorTools.measurement
                        x = trackingParams.all_measure[i][0]
                        y = trackingParams.all_measure[i][1]
                        ctx.arc(x, y, 3, 0, Math.PI * 2, false);
                        ctx.fill();
                    }
                } else {
                    for(i = 0; i < trackingParams.measurements.length; i++) {
                        ctx.beginPath();
                        ctx.fillStyle = colorTools.measurement
                        x = trackingParams.measurements[i][0]
                        y = trackingParams.measurements[i][1]
                        ctx.arc(x, y, 3, 0, Math.PI * 2, false);
                        ctx.fill();
                    }
                }
            }

            ctx.beginPath();
            ctx.fillStyle = ctx.fillStyle = Qt.rgba(0.3, 0.3, 0.3, 1);
            ctx.rect(parent.width - 180, 20, 150, 170);
            ctx.fill();

            const TEXT_START_X = parent.width - 170;
            const SYM_START_X  = parent.width - 50;
            var text_start_y = 20;
            ctx.font = '16px Verdana'


            text_start_y += 30;
            drawLegends(ctx, text_start_y, colorTools.truth, "Truth", TEXT_START_X, text_start_y, SYM_START_X);
            text_start_y += 30;
            drawLegends(ctx, text_start_y, colorTools.measurement, "Measure", TEXT_START_X, text_start_y, SYM_START_X);
            text_start_y += 30;
            drawLegends(ctx, text_start_y, colorTools.nearest_neighbor, "NN", TEXT_START_X, text_start_y, SYM_START_X);
            text_start_y += 30;
            drawLegends(ctx, text_start_y, colorTools.pda, "PDA", TEXT_START_X, text_start_y, SYM_START_X);
            text_start_y += 30;
            drawLegends(ctx, text_start_y, colorTools.gaussian_sum, "GS", TEXT_START_X, text_start_y, SYM_START_X);

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

    Connections {

        target: backend

        onResetData: {
            trackingParams.nearest_neighbor = []
            trackingParams.pda              = []
            trackingParams.gaussian_sum     = []
            trackingParams.line             = []
            trackingParams.all_measure      = []
        }

        onSingleTrackingAddItem: {
            if(typeOfItem.valueOf() === "nearest_neighbor") {
                trackingParams.nearest_neighbor.push( [ parseInt(x), parseInt(y) ] )
            } else if(typeOfItem.valueOf() === "pda") {
                trackingParams.pda.push( [ parseInt(x), parseInt(y) ] )
            } else if(typeOfItem.valueOf() === "gaussian_sum") {
                trackingParams.gaussian_sum.push( [ parseInt(x), parseInt(y) ] )
            } else if(typeOfItem.valueOf() === "ground_truth") {
                trackingParams.line.push( [ parseInt(x), parseInt(y) ] )
            } else if(typeOfItem.valueOf() === "repaint") {
                canvas.requestPaint()
            }
        }

        onSingleTrackingAddData: {
            trackingParams.measurements = []
            for(var i = 0; i < x.length; i++) {
                trackingParams.measurements.push( [ parseInt(x[i]), parseInt(y[i]) ] )
                trackingParams.all_measure.push( [ parseInt(x[i]), parseInt(y[i]) ] )
            }
            canvas.requestPaint()
        }
    }

}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:800}
}
##^##*/
