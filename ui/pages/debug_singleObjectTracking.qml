import QtQuick 2.12
import QtQml 2.12
import QtQuick.Controls 2.12
import "../controls"


Item {

    QtObject {
        id: trackingParams
        property variant points:                    []
        property variant line:                      []
        property variant measurements:              []
        property variant all_measure:               []
        property variant nearest_neighbor:          []
        property variant pda:                       []
        property variant gaussian_sum:              []
        property bool show_nearest_neighbor:        true
        property bool show_pda:                     true
        property bool show_gaussian_sum:            true
        property bool show_measurement:             true
        property bool show_all_meas:                false
        property bool show_truth:                   true
    }

    function drawPoints(ctx, items, color, radius=5, should_fill=false) {

        var x = 0;
        var y = 0;
        for(var i = 0; i < items.length; i++) {
            ctx.beginPath();
            if(should_fill === true)    ctx.fillStyle = color
            else                        ctx.strokeStyle = color
            x = items[i][0]
            y = items[i][1]
            ctx.arc(x, y, radius, 0, Math.PI * 2, false);
            if(should_fill === true)    ctx.fill();
            else                        ctx.stroke();
        }
    }

    function drawLegend(ctx, text_start_y, color, txt, TEXT_START_X, text_start_y, SYM_START_X) {

        ctx.beginPath();
        ctx.fillStyle = color;
        ctx.fillText(txt, TEXT_START_X, text_start_y);
        ctx.arc(SYM_START_X, text_start_y - 5, 5, 0, Math.PI * 2, false);
        ctx.fill()
    }

    function drawLegends(ctx) {

        ctx.beginPath();
        ctx.fillStyle = ctx.fillStyle = Qt.rgba(0.3, 0.3, 0.3, 1);
        ctx.rect(parent.width - 180, 20, 150, 170);
        ctx.fill();

        const TEXT_START_X = parent.width - 170;
        const SYM_START_X  = parent.width - 50;
        var text_start_y = 20;
        ctx.font = '16px Verdana'

        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.truth, "Truth", TEXT_START_X, text_start_y, SYM_START_X);
        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.measurement, "Measure", TEXT_START_X, text_start_y, SYM_START_X);
        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.nearest_neighbor, "NN", TEXT_START_X, text_start_y, SYM_START_X);
        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.pda, "PDA", TEXT_START_X, text_start_y, SYM_START_X);
        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.gaussian_sum, "GS", TEXT_START_X, text_start_y, SYM_START_X);
    }

    QtObject {
        id: stateMachine
        readonly property int draw_line :  1
        property int state: draw_line
    }

    Row {

        id: colorTools
        height: btnRun.height
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
            id: btnRun
            btnText: "Run"
            btnIconSource: "../../ui/images/svg_images/run.svg"
            onClicked: {
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
            checked: false

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

            if(trackingParams.show_measurement == true) {

                if(trackingParams.show_all_meas == true) {
                    drawPoints(ctx, trackingParams.all_measure, colorTools.measurement, 3, true)
                } else {
                    drawPoints(ctx, trackingParams.measurements, colorTools.measurement, 3, true)
                }
            }

            if(trackingParams.show_nearest_neighbor == true)    drawPoints(ctx, trackingParams.nearest_neighbor, colorTools.nearest_neighbor)
            if(trackingParams.show_pda == true)                 drawPoints(ctx, trackingParams.pda, colorTools.pda)
            if(trackingParams.show_gaussian_sum == true)        drawPoints(ctx, trackingParams.gaussian_sum, colorTools.gaussian_sum)

            drawLegends(ctx);
        }

        MouseArea {
            id: area
            anchors.fill: parent
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
