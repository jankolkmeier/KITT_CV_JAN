var app = angular.module('remoteControlApp', ['luegg.directives']);
window._stream = false;

app.factory('socket', function($rootScope) {
    var socket = new eio.Socket();
    return {
        on: function (eventName, callback) {
          socket.on(eventName, function () {  
            var args = arguments;
            $rootScope.$apply(function () {
              callback.apply(socket, args);
            });
          });
        },
        send: function(data) {
            socket.send(data);
        },
        emit: function (eventName, data, callback) {
          socket.emit(eventName, data, function () {
            var args = arguments;
            $rootScope.$apply(function () {
              if (callback) {
                callback.apply(socket, args);
              }
            });
          })
        }
    };
});

function RemoteCTRL($scope, socket) {
    $scope.timers = {};
    $scope.xpos  = 0;
    $scope.ypos  = 0;
    $scope.zpos  = 0;
    $scope.glued = true;

    $scope.log = [];
    $scope.img = "iVBORw0KGgoAAAANSUhEUgAAAAIAAAABCAYAAAD0In+KAAAABmJLR0QA/wD/AP+gvaeTAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAB3RJTUUH3gYDDC8yqsQdOAAAAB1pVFh0Q29tbWVudAAAAAAAQ3JlYXRlZCB3aXRoIEdJTVBkLmUHAAAAC0lEQVQI12NggAIAAAkAAWMqFg0AAAAASUVORK5CYII=";
    $scope.query = "";
    $scope.stream = false;

    $scope.sendQuery = function() {
        socket.send($scope.query);
    }

    $scope.send = function(msg) {
        socket.send(msg);
    }

    $scope.getParams = function() {
        socket.send('GET PARAMS');
    };

    $scope.getFrame = function() {
        socket.send('GET IMG');
    };


    socket.on('message', function(data){
        var msg = JSON.parse(data);
        console.log(msg);
        if (msg.param == 'IMG') {
            //$scope.img = "data:image/png;base64,"+msg.value;
            $scope.img = msg.value;

            //document.getElementById("debug_img").src = "data:image/png;base64,"+msg.value;
            if ($scope.stream) {
                socket.send("GET IMG");
                //socket.send("GET profile");
            }
        } else if (msg.param == 'profile') {
            $scope.timers = {};
            var timers = msg.value.split("\n");
            for (var i = 0; i < timers.length; i++) {
                var timer = timers[i].split(":");
                if (timer.length == 2) {
                    $scope.timers[timer[0]] = timer[1];
                }
            }
        } else if (msg.param == 'pos') {
            var poss = msg.value.split(" ");
            $scope.xpos = poss[0];
            $scope.ypos = poss[1];
            $scope.zpos = poss[2];
        } else if (msg.param == 'log') {
            $scope.log.push({ txt: msg.value+"", class: 'log_reg' });
        } else {
            $scope.log.push({ txt: data, class: 'log_war' });
        }
    });

    socket.on('close', function(){
    });
    socket.on('open', function(){
    });
}
