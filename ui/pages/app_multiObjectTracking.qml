import QtQuick 2.12
import QtQml 2.12
import QtQuick.Controls 2.12
import QtQuick.Dialogs 1.1
import QtQuick.Layouts 1.12
import QtQuick.Controls.Styles 1.4
import "../controls"

Item {

    MessageDialog {
        id: messageDialog
        title: "Warning"
        icon: StandardIcon.Warning
        onAccepted: {
        }
    }

    QtObject {
        id: tracking_params
        property variant points:                []
        property variant curr_points:           []
        property variant clutters:              []
        property variant measurements:          []
        property variant gnn:                   []
        property variant jpda:                  []
        property variant mht:                   []
        property int  number_of_birth:          0
        property int  line_pointer:             0
        property int  lambda_c:                 0
        property bool show_gnn:                 true
        property bool show_jpda:                true
        property bool show_mht:                 true
        property bool show_measurement:         true
        property bool show_all_meas:            true
        property bool show_truth:               true
    }

    Timer {
        id: timer
        interval: 15;
        running: false;
        repeat: false
        onTriggered: {
            tracking_params.line_pointer += 1
            canvas.requestPaint()
        }
    }

    function drawPoints(ctx, items, color) {

        var x, y;
        for(var i = 0; i < items.length; i++) {
            ctx.beginPath();
            ctx.fillStyle = color
            x = items[i][0]
            y = items[i][1]
            ctx.arc(x, y, 3, 0, Math.PI * 2, false);
            ctx.fill();
        }
    }

    function drawLines(ctx, items, color) {

        var x, y;
        for(var i = 0; i < items.length - 1; i++) {
            ctx.lineWidth = 3
            ctx.strokeStyle = color
            ctx.beginPath()
            x = items[i][0]
            y = items[i][1]
            ctx.moveTo(x, y)
            x = items[i + 1][0]
            y = items[i + 1][1]
            ctx.lineTo(x, y)
            ctx.stroke()
        }
    }

    function drawArrPoints(ctx, items, color, show_start, show_len, radius=5, should_fill=false) {

        var x, y;
        for(var i = 0; i < items.length; i++) {
            var len = Math.min(items[i].length, show_len)
            for(var j = show_start; j < len; j++) {
                ctx.beginPath();
                if(should_fill === true)    ctx.fillStyle = color
                else                        ctx.strokeStyle = color
                x = items[i][j][0]
                y = items[i][j][1]
                ctx.arc(x, y, radius, 0, Math.PI * 2, false);
                if(should_fill === true)    ctx.fill();
                else                        ctx.stroke();
            }
        }
    }

    function drawLimitedPoints(ctx, items, color, ref_points, radius=5, should_fill=false) {

        var x, y;
        for(var i = 0; i < items.length; i++) {
            var len = Math.min(items[i].length, ref_points[i].length)
            for(var j = 0; j < len; j++) {
                ctx.beginPath();
                if(should_fill === true)    ctx.fillStyle = color
                else                        ctx.strokeStyle = color
                x = items[i][j][0]
                y = items[i][j][1]
                ctx.arc(x, y, radius, 0, Math.PI * 2, false);
                if(should_fill === true)    ctx.fill();
                else                        ctx.stroke();
            }
        }
    }

    function drawArrLines(ctx, items, color) {

        var x, y;
        for(var i = 0; i < items.length; i++) {
            var curr_line = items[i]
            for(var j = 0; j < curr_line.length - 1; j++) {
                ctx.lineWidth = 3
                ctx.strokeStyle = color
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

    function shuffle(array) {

        let currentIndex = array.length,  randomIndex;

        while (currentIndex !== 0) {

            randomIndex = Math.floor(Math.random() * currentIndex);
            currentIndex--;

            [array[currentIndex], array[randomIndex]] = [array[randomIndex], array[currentIndex]];
        }

        return array;
    }

    function correctMeasurements(items) {


        for(var j = 0; j < items.length; j++) {

            var points = items[j];
            while(true) {

                var found_gap = false;
                for(var i = 0; i < points.length-1; i++) {
                    var x1 = points[i][0];
                    var y1 = points[i][1];
                    var x2 = points[i+1][0];
                    var y2 = points[i+1][1];
                    var d = Math.sqrt(Math.pow(x2-x1, 2) + Math.pow(y2-y1, 2));
                    if(d > 4) {
                        points.splice(i+1, 0, [(x1+x2)/2, (y1+y2)/2]);
                        found_gap = true;
                    }
                }
                if(found_gap === false)
                    break;
            }
        }
        canvas.requestPaint()
    }

    function sendMeasurement(measurements, clutters, lambda_c, curr_pointer) {

        var start_idx = parseInt(lambda_c * (curr_pointer-1));
        var end_idx = parseInt(lambda_c * curr_pointer);
        var curr_clutter = clutters.slice(start_idx, end_idx)
        var xs = [];
        var ys = [];
        for(var j = 0; j < curr_clutter.length; j++) {
            xs.push(curr_clutter[j][0]);
            ys.push(curr_clutter[j][1]);
        }

        for(var i = 0; i < tracking_params.number_of_birth; i++) {
            if(curr_pointer < measurements[i].length) {
                xs.push(measurements[i][curr_pointer][0]);
                ys.push(measurements[i][curr_pointer][1]);
            }
        }

        backend.multiMeasurements(xs, ys);
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
        id:                                     state_machine
        readonly property int draw_line:        1
        readonly property int clutter:          2
        readonly property int timer_running:    3
        readonly property int timer_end:        4
        property int state:                     draw_line
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

                if(state_machine.state === state_machine.timer_running) {
                    return;
                } else if(state_machine.state === state_machine.timer_end) {
                    state_machine.state = state_machine.draw_line;
                }

                tracking_params.points          = []
                tracking_params.curr_points     = []
                tracking_params.clutters        = []
                tracking_params.measurements    = []
                tracking_params.gnn             = []
                tracking_params.jpda            = []
                tracking_params.mht             = []
                canvas.requestPaint()
            }
        }
        ButtonLabel {
            id: btnDrawLine
            btnText: "draw"
            btnIconSource: "../../ui/images/svg_images/draw_line.svg"
            onClicked: {
                if(state_machine.state == state_machine.clutter) {
                    state_machine.state = state_machine.draw_line
                }
            }
        }
        ButtonLabel {
            id: btnClutter
            btnText: "clutter"
            btnIconSource: "../../ui/images/svg_images/clutter.svg"
            onClicked: {
                if(state_machine.state == state_machine.draw_line) {
                    state_machine.state = state_machine.clutter
                }
            }
        }
        ButtonLabel {
            id: btnMeasurements
            btnText: "measure"
            btnIconSource: "../../ui/images/svg_images/measurement.svg"
            onClicked: {

                if(state_machine.state === state_machine.draw_line || state_machine.state === state_machine.clutter) {
                    if(tracking_params.points.length === 0) {
                        messageDialog.text = "First You have to Draw a Path";
                        messageDialog.visible = true;
                        return;
                    }

                    tracking_params.measurements = []
                    for(var i = 0; i < tracking_params.points.length; i++) {
                        var points = tracking_params.points[i]
                        var curr_measurements = []
                        for(var j = 0; j < points.length; j++) {
                            var coeff = (j < 50) ? 6 : 12;
                            var x = points[j][0] + (Math.random() - 0.5) * coeff;
                            var y = points[j][1] + (Math.random() - 0.5) * coeff;
                            curr_measurements.push([x, y])
                        }
                        tracking_params.measurements.push(curr_measurements)
                    }
                    canvas.requestPaint()
                }
            }
        }
        ButtonLabel {
            id: btnRun
            btnText: "Run"
            btnIconSource: "../../ui/images/svg_images/run.svg"
            onClicked: {

                if(tracking_params.points.length === 0) {
                    messageDialog.text = "There is no Data to Track";
                    messageDialog.visible = true;
                    return;
                }
                else if(tracking_params.measurements.length === 0) {
                    messageDialog.text = "There is no Measurement";
                    messageDialog.visible = true;
                    return;
                }
                else if(state_machine.state === state_machine.timer_running) {
                    return;
                }

                tracking_params.number_of_birth = tracking_params.points.length
                tracking_params.lambda_c = parseInt(tracking_params.clutters.length / tracking_params.measurements.length);

                var x1 = [];
                var y1 = [];
                var x2 = [];
                var y2 = [];
                for(var i = 0; i < tracking_params.number_of_birth; i++) {
                    x1.push(tracking_params.points[i][0][0]);
                    y1.push(tracking_params.points[i][0][1]);
                    x2.push(tracking_params.points[i][1][0]);
                    y2.push(tracking_params.points[i][1][1]);
                }

                backend.initMultiTrackers(x1, y1, x2, y2, tracking_params.number_of_birth, tracking_params.lambda_c);
                if(timer.running === false) {
                    tracking_params.gnn             = []
                    tracking_params.jpda            = []
                    tracking_params.mht             = []
                    tracking_params.line_pointer    = 0;
                    tracking_params.clutters        = shuffle(tracking_params.clutters);
                    for(i = 0; i < tracking_params.number_of_birth; i++) {
                        tracking_params.gnn[i]       = []
                        tracking_params.jpda[i]      = []
                        tracking_params.mht[i]       = []
                    }

                    state_machine.state = state_machine.timer_running;
                    //canvas.requestPaint()
                    timer.start();
                }
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
                tracking_params.show_gnn = checked
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
                tracking_params.show_jpda = checked
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
                tracking_params.show_mht = checked
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
                tracking_params.show_measurement = checked
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
                tracking_params.show_all_meas = checked
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
                tracking_params.show_truth = checked
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
            //console.log("onPaint")
            var ctx = getContext('2d')

            ctx.fillStyle = Qt.rgba(0.5, 0.5, 0.5, 1);
            ctx.fillRect(0, 0, width, height);

            var curr_pointer = 0;
            var max_points = 0;
            if(tracking_params.points.length > 0) {
                max_points = tracking_params.points[0].length - 1;
                for(var i = 0; i < tracking_params.points.length; i++) {
                    if(max_points < tracking_params.points[i].length - 1)
                        max_points = tracking_params.points[i].length - 1;
                }
                curr_pointer = Math.min(tracking_params.line_pointer, max_points);
                //console.log("max_points: ", max_points);
            }
            var lambda_c = 0;
            if(tracking_params.measurements.length > 0)
                lambda_c = tracking_params.clutters.length / tracking_params.measurements[0].length;

            if(state_machine.state === state_machine.timer_running &&
                    tracking_params.line_pointer >= max_points) {
                timer.stop();
                state_machine.state = state_machine.timer_end;
                console.log(max_points, tracking_params.line_pointer, curr_pointer)
                for(var j = 0; j < tracking_params.number_of_birth; j++) {
                    console.log(tracking_params.points[j].length, tracking_params.measurements[j].length,
                                tracking_params.gnn[j].length,tracking_params.jpda[j].length, tracking_params.mht[j].length)
                }
            }

            drawLines(ctx, tracking_params.curr_points, canvas.color);

            if(tracking_params.show_truth == true) {
                drawArrLines(ctx, tracking_params.points, canvas.color)
                //drawArrPoints(ctx, tracking_params.points, colorTools.truth, 5)
            }


            if(tracking_params.show_gnn   == true) drawLimitedPoints(ctx, tracking_params.gnn, colorTools.gnn, tracking_params.points)
            if(tracking_params.show_jpda  == true) drawLimitedPoints(ctx, tracking_params.jpda, colorTools.jpda,  tracking_params.points)
            if(tracking_params.show_mht   == true) drawLimitedPoints(ctx, tracking_params.mht, colorTools.mht,  tracking_params.points)

            //if(tracking_params.show_gnn   == true) drawArrPoints(ctx, tracking_params.gnn, colorTools.gnn, 0, curr_pointer)
            //if(tracking_params.show_jpda  == true) drawArrPoints(ctx, tracking_params.jpda, colorTools.jpda, 0, curr_pointer)
            //if(tracking_params.show_mht   == true) drawArrPoints(ctx, tracking_params.mht, colorTools.mht, 0, curr_pointer)

            if(tracking_params.show_measurement == true) {

                if(tracking_params.show_all_meas == true) {
                    drawArrPoints(ctx, tracking_params.measurements, colorTools.measurement, 0, curr_pointer, 3, true)
                    drawPoints(ctx, tracking_params.clutters.slice(0, parseInt(lambda_c * curr_pointer)), colorTools.measurement, 3, true)
                } else {

                    if(state_machine.state === state_machine.timer_running || state_machine.state === state_machine.timer_end) {
                        if(curr_pointer > 0) {
                            drawPoints(ctx, tracking_params.clutters.slice(parseInt(lambda_c * (curr_pointer-1)),
                                                                           parseInt(lambda_c * curr_pointer)), colorTools.measurement, 3, true)
                            drawArrPoints(ctx, tracking_params.measurements, colorTools.measurement, curr_pointer, curr_pointer+1, 3, true)
                            //if(curr_pointer < max_points - 10)
                            //    drawArrPoints(ctx, tracking_params.measurements, colorTools.measurement, curr_pointer, curr_pointer+10, 3, true)
                            //else
                            //    drawArrPoints(ctx, tracking_params.measurements, colorTools.measurement,curr_pointer, max_points-1, 3, true)
                        }
                    } else {
                        drawArrPoints(ctx, tracking_params.measurements, colorTools.measurement, 0, curr_pointer, 3, true);
                        drawPoints(ctx, tracking_params.clutters, colorTools.measurement, 3, true);
                    }
                }
            }

            drawLegends(ctx);

            if(state_machine.state === state_machine.timer_running) {
                //console.log("Send Measurement")
                sendMeasurement(tracking_params.measurements, tracking_params.clutters, lambda_c, curr_pointer)
            }
        }

        MouseArea {
            id: area
            anchors.fill: parent
            onPressed: {
                if(state_machine.state == state_machine.draw_line) {
                    tracking_params.curr_points = []
                    tracking_params.curr_points.push( [ parseInt(mouseX), parseInt(mouseY) ] )

                } else if(state_machine.state == state_machine.clutter) {
                    var x = parseInt( mouseX )
                    var y = parseInt( mouseY )
                    for(var i = 0; i < 170; i++) {
                        var currX = x + (Math.random() - 0.5) * 400
                        var currY = y + (Math.random() - 0.5) * 400
                        tracking_params.clutters.push( [ currX, currY ] )
                    }
                    canvas.requestPaint()
                }
            }
            onReleased: {
                if(state_machine.state === state_machine.draw_line) {
                    tracking_params.points.push( tracking_params.curr_points )
                    correctMeasurements(tracking_params.points)
                    var max_points = tracking_params.points[0].length - 1;
                    for(var i = 0; i < tracking_params.points.length; i++) {
                        if(max_points < tracking_params.points[i].length - 1)
                            max_points = tracking_params.points[i].length - 1;
                    }
                    tracking_params.line_pointer = max_points;
                    tracking_params.curr_points = []
                }
            }
            onPositionChanged: {
                if(state_machine.state == state_machine.draw_line) {
                    tracking_params.curr_points.push( [ parseInt(mouseX), parseInt(mouseY) ] )
                }
                canvas.requestPaint()
            }
        }
    }

    Connections {

        target: backend

        onMultiTrackingAddItem: {
            if(typeOfItem.valueOf() === "gnn") {
                for(var i = 0; i < tracking_params.number_of_birth; i++) {
                    tracking_params.gnn[i].push( [ parseInt(x[i]), parseInt(y[i]) ] )
                }
            } else if(typeOfItem.valueOf() === "jpda") {
                for(i = 0; i < tracking_params.number_of_birth; i++) {
                    tracking_params.jpda[i].push( [ parseInt(x[i]), parseInt(y[i]) ] )
                }
            } else if(typeOfItem.valueOf() === "mht") {
                for(i = 0; i < tracking_params.number_of_birth; i++) {
                    tracking_params.mht[i].push( [ parseInt(x[i]), parseInt(y[i]) ] )
                }
            } else if(typeOfItem.valueOf() === "repaint") {
                //console.log("repaint")
                timer.start();
                //tracking_params.line_pointer += 1
                //canvas.requestPaint()
            }
        }
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:800}
}
##^##*/
