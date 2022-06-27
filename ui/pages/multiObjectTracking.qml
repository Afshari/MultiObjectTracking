import QtQuick 2.12
import QtQml 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import QtQuick.Controls.Styles 1.4
import "../controls"

Item {

    QtObject {
        id: trackingParams
        property variant points:                []
        property variant curr_points:           []
        property variant lines:                 []
        property variant curr_line:             []
        property variant clutters:              []
        property variant measurements:          []
        property variant gnn:                   Array(4)
        property variant jpda:                  Array(4)
        property variant mht:                   Array(4)
        property int number_of_birth:           0
        property bool showGNN:                  true
        property bool showJPDA:                 true
        property bool showMHT:                  true
        property bool showMeasurement:          true
        property bool showTruth:                true
    }

    function drawPoints(ctx, items, color) {

        var x = 0;
        var y = 0;
        for(var i = 0; i < trackingParams.number_of_birth; i++) {
            for(var j = 0; j < items[i].length; j++) {
                ctx.beginPath();
                ctx.strokeStyle = color
                x = items[i][j][0]
                y = items[i][j][1]
                ctx.arc(x, y, 5, 0, Math.PI * 2, false);
                ctx.stroke();
            }
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
        property color truth:           "#33B5E5"
        property color gnn:             "#FF0000"
        property color jpda:            "#00FF00"
        property color mht:             "#0000FF"
        property color measurement:     "white"


        ButtonLabel {
            id: btnClear
            btnText: "erase"
            btnIconSource: "../../ui/images/svg_images/eraser.svg"
            onClicked: {
                trackingParams.points           = []
                trackingParams.curr_points      = []
                trackingParams.lines            = []
                trackingParams.curr_line        = []
                trackingParams.clutters         = []
                trackingParams.measurements     = []
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
                    var points = trackingParams.points[i]
                    var curr_measurements = []
                    for(var j = 0; j < points.length; j++) {
                        var x = points[j][0] + (Math.random() - 0.5) * 30
                        var y = points[j][1] + (Math.random() - 0.5) * 30
                        curr_measurements.push( [x, y] )
                    }
                    trackingParams.measurements.push( curr_measurements )
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
//                var xs = [];
//                var ys = [];
//                for(var i = 0; i < trackingParams.measurements.length; i++) {
//                    xs.push( trackingParams.measurements[i][0] )
//                    ys.push( trackingParams.measurements[i][1] )
//                }
                // backend.getMeasurements( xs, ys )
                // backend.qmlCommand("multi");
                backend.qmlCommand("jpda");
                // backend.qmlCommand("gnn");
            }
        }
        ButtonLabel {
            id: btnStop
            btnText: "Sop"
            btnIconSource: "../../ui/images/svg_images/run.svg"
            onClicked: {
                backend.qmlCommand("stop");
            }
        }


        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "GNN"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                trackingParams.showGNN = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "JPDA"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                trackingParams.showJPDA = checked
                canvas.requestPaint()
            }
        }
        CustomSwitch {
            height: parent.height - 5
            checked: true

            Label {
                text: "MHT"
                color: "white"
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: parent.bottom
            }
            onCheckedChanged: {
                trackingParams.showMHT = checked
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
                trackingParams.showMeasurement = checked
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
                trackingParams.showTruth = checked
                canvas.requestPaint()
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
        property color color: colorTools.truth

        onPaint: {
            // console.log("onPaint")
            var ctx = getContext('2d')

            ctx.fillStyle = Qt.rgba(0.5, 0.5, 0.5, 1);
            ctx.fillRect(0, 0, width, height);

            var x = 0
            var y = 0
            for(var i = 0; i < trackingParams.curr_line.length - 1; i++) {
                ctx.lineWidth = 3
                ctx.strokeStyle = canvas.color
                ctx.beginPath()
                x = trackingParams.curr_line[i][0]
                y = trackingParams.curr_line[i][1]
                ctx.moveTo(x, y)
                x = trackingParams.curr_line[i + 1][0]
                y = trackingParams.curr_line[i + 1][1]
                ctx.lineTo(x, y)
                ctx.stroke()
            }

            if(trackingParams.showMeasurement == true) {
                for(i = 0; i < trackingParams.clutters.length; i++) {
                    ctx.beginPath();
                    ctx.fillStyle = colorTools.measurement
                    x = trackingParams.clutters[i][0]
                    y = trackingParams.clutters[i][1]
                    ctx.arc(x, y, 3, 0, Math.PI * 2, false);
                    ctx.fill();
                }
                for(i = 0; i < trackingParams.measurements.length; i++) {
                    var curr_measurements = trackingParams.measurements[i]
                    for(var j = 0; j < curr_measurements.length; j++) {
                        ctx.beginPath();
                        ctx.fillStyle = colorTools.measurement
                        x = curr_measurements[j][0]
                        y = curr_measurements[j][1]
                        ctx.arc(x, y, 3, 0, Math.PI * 2, false);
                        ctx.fill();
                    }
                }
            }


            if(trackingParams.showTruth == true) {
                for(i = 0; i < trackingParams.lines.length; i++) {
                    var curr_line = trackingParams.lines[i]
                    for(j = 0; j < curr_line.length - 1; j++) {
                        ctx.lineWidth = 3
                        ctx.strokeStyle = canvas.color
                        ctx.beginPath()
                        x = curr_line[j][0]
                        y = curr_line[j][1]
                        ctx.moveTo(x, y)
                        x = curr_line[j + 1][0]
                        y = curr_line[j + 1][1]
                        ctx.lineTo(x, y)
                        ctx.stroke()
                    }
                }
            }


            if(trackingParams.showGNN   == true) drawPoints(ctx, trackingParams.gnn, colorTools.gnn)
            if(trackingParams.showJPDA  == true) drawPoints(ctx, trackingParams.jpda, colorTools.jpda)
            if(trackingParams.showMHT   == true) drawPoints(ctx, trackingParams.mht, colorTools.mht)


            for(i = 0; i < trackingParams.points.length; i++) {
                var curr_points = trackingParams.points[i]
                for(j = 0; j < curr_points.length; j++) {
                    ctx.beginPath();
                    ctx.fillStyle = colorTools.truth
                    x = curr_points[j][0]
                    y = curr_points[j][1]
                    ctx.arc(x, y, 5, 0, Math.PI * 2, false);
                    ctx.fill();
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
            drawLegends(ctx, text_start_y, colorTools.gnn, "GNN", TEXT_START_X, text_start_y, SYM_START_X);
            text_start_y += 30;
            drawLegends(ctx, text_start_y, colorTools.jpda, "JPDA", TEXT_START_X, text_start_y, SYM_START_X);
            text_start_y += 30;
            drawLegends(ctx, text_start_y, colorTools.mht, "MHT", TEXT_START_X, text_start_y, SYM_START_X);

        }

        MouseArea {
            id: area
            anchors.fill: parent
            onPressed: {
                if(stateMachine.state == stateMachine.draw_line) {
                    trackingParams.curr_points = []
                    trackingParams.curr_points.push( [ parseInt(mouseX), parseInt(mouseY) ] )
                    trackingParams.curr_line = []
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
                trackingParams.points.push( trackingParams.curr_points )
                trackingParams.curr_points = []
                trackingParams.lines.push( trackingParams.curr_line )
                trackingParams.curr_line = []
            }
            onPositionChanged: {
                if(stateMachine.state == stateMachine.draw_line) {
                    trackingParams.curr_line.push( [parseInt(mouseX), parseInt(mouseY)] )
                    var lastX = trackingParams.curr_points[trackingParams.curr_points.length - 1][0]
                    var lastY = trackingParams.curr_points[trackingParams.curr_points.length - 1][1]
                    if( Math.hypot(lastX - mouseX, lastY - mouseY) > 40) {
                        trackingParams.curr_points.push( [ parseInt(mouseX), parseInt(mouseY) ] )
                    }
                }
                canvas.requestPaint()
            }
        }
    }

    Connections {

        target: backend

        onSetNumberOfBirth: {
            trackingParams.number_of_birth  = nbirths
            trackingParams.gnn              = Array(trackingParams.number_of_birth)
            trackingParams.jpda             = Array(trackingParams.number_of_birth)
            trackingParams.mht              = Array(trackingParams.number_of_birth)
            trackingParams.lines            = Array(trackingParams.number_of_birth)
            for(var i = 0; i < trackingParams.number_of_birth; i++) {
                trackingParams.gnn[i]       = []
                trackingParams.jpda[i]      = []
                trackingParams.mht[i]       = []
                trackingParams.lines[i]     = []
            }
        }

        onMultiTrackingAddItem: {
            if(typeOfItem.valueOf() === "gnn") {
                for(var i = 0; i < trackingParams.number_of_birth; i++) {
                    trackingParams.gnn[i].push( [ parseInt(x[i]), parseInt(y[i]) ] )
                }
            } else if(typeOfItem.valueOf() === "jpda") {
                for(i = 0; i < trackingParams.number_of_birth; i++) {
                    trackingParams.jpda[i].push( [ parseInt(x[i]), parseInt(y[i]) ] )
                }
            } else if(typeOfItem.valueOf() === "mht") {
                for(i = 0; i < trackingParams.number_of_birth; i++) {
                    trackingParams.mht[i].push( [ parseInt(x[i]), parseInt(y[i]) ] )
                }
            } else if(typeOfItem.valueOf() === "ground_truth") {
                for(i = 0; i < trackingParams.number_of_birth; i++) {
                    trackingParams.lines[i].push( [ parseInt(x[i]), parseInt(y[i]) ] )
                }
            } else if(typeOfItem.valueOf() === "repaint") {
                canvas.requestPaint()
            }
        }
        onMultiTrackingAddData: {
            // trackingParams.clutters = []
            for(var i = 0; i < x.length; i++) {
                trackingParams.clutters.push( [ parseInt(x[i]), parseInt(y[i]) ] )
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
