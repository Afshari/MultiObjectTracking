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
        property variant all_measures:          []
        property variant gnn:                   Array(4)
        property variant jpda:                  Array(4)
        property variant mht:                   Array(4)
        property int number_of_birth:           0
        property bool show_gnn:                 true
        property bool show_jpda:                true
        property bool show_mht:                 true
        property bool show_measurement:         true
        property bool show_all_meas:            false
        property bool show_truth:               true
    }

    function drawPointsStroke(ctx, items, color) {

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

    function drawPointsFill(ctx, items, color) {

        var x = 0;
        var y = 0;
        for(var i = 0; i < items.length; i++) {
            ctx.beginPath();
            ctx.fillStyle = color
            x = items[i][0]
            y = items[i][1]
            ctx.arc(x, y, 3, 0, Math.PI * 2, false);
            ctx.fill();
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
        drawLegend(ctx, text_start_y, colorTools.gnn, "GNN", TEXT_START_X, text_start_y, SYM_START_X);
        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.jpda, "JPDA", TEXT_START_X, text_start_y, SYM_START_X);
        text_start_y += 30;
        drawLegend(ctx, text_start_y, colorTools.mht, "MHT", TEXT_START_X, text_start_y, SYM_START_X);
    }

    QtObject {
        id: stateMachine
        readonly property int draw_line :  1
        readonly property int clutter   :  2
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
        property color truth:           "#33B5E5"
        property color gnn:             "#FF0000"
        property color jpda:            "#00FF00"
        property color mht:             "#0000FF"
        property color measurement:     "white"


        ButtonLabel {
            id: btnRun
            btnText: "Run"
            btnIconSource: "../../ui/images/svg_images/run.svg"
            onClicked: {
                 backend.qmlCommand("multi");
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
                trackingParams.show_gnn = checked
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
                trackingParams.show_jpda = checked
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
                trackingParams.show_mht = checked
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

            var ctx = getContext('2d')

            ctx.fillStyle = Qt.rgba(0.5, 0.5, 0.5, 1);
            ctx.fillRect(0, 0, width, height);

            var i, j;
            var x = 0;
            var y = 0;
            for(i = 0; i < trackingParams.curr_line.length - 1; i++) {
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

            if(trackingParams.show_measurement == true) {

                if(trackingParams.show_all_meas == true) {
                    drawPointsFill(ctx, trackingParams.all_measures, colorTools.measurement)
                } else {
                    drawPointsFill(ctx, trackingParams.measurements, colorTools.measurement)
                }
            }


            if(trackingParams.show_truth == true) {
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


            if(trackingParams.show_gnn   == true) drawPointsStroke(ctx, trackingParams.gnn, colorTools.gnn)
            if(trackingParams.show_jpda  == true) drawPointsStroke(ctx, trackingParams.jpda, colorTools.jpda)
            if(trackingParams.show_mht   == true) drawPointsStroke(ctx, trackingParams.mht, colorTools.mht)


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

            drawLegends(ctx);
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
            trackingParams.all_measures     = []
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
            trackingParams.measurements = []
            for(var i = 0; i < x.length; i++) {
                trackingParams.all_measures.push( [ parseInt(x[i]), parseInt(y[i]) ] )
                trackingParams.measurements.push( [ parseInt(x[i]), parseInt(y[i]) ] )
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
