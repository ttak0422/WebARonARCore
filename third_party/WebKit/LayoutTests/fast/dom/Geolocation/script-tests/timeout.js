description("Tests that when timeout is non-zero, the success callback is called as expected.");

var mockLatitude = 51.478;
var mockLongitude = -0.166;
var mockAccuracy = 100.0;

var position;

geolocationServiceMock.then(mock => {
    mock.setGeolocationPermission(true);
    mock.setGeolocationPosition(mockLatitude,
                                mockLongitude,
                                mockAccuracy);

    navigator.geolocation.getCurrentPosition(function(p) {
        position = p;
        shouldBe('position.coords.latitude', 'mockLatitude');
        shouldBe('position.coords.longitude', 'mockLongitude');
        shouldBe('position.coords.accuracy', 'mockAccuracy');
        finishJSTest();
    }, function(e) {
        testFailed('Error callback invoked unexpectedly');
        finishJSTest();
    }, {
        timeout: 1000
    });
});

window.jsTestIsAsync = true;
