
var map = L.map('map').setView([37.9838, 23.7275], 123); // map with attica greece as its center

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { //load map tiles from osm and add to map
    attribution: '&copy; OpenStreetMap contributors'
}).addTo(map);

var markers = []; //store map markers
var selectedPoints = [];// and selected coords

function onMapClick(e) {
    if (markers.length >= 2) { // if more than two markers are put
        map.removeLayer(markers[0]); //the oldest one is removed
        markers.shift();//the other becomes the first
        selectedPoints.shift();//the older point of the marker is removed from the selected
    }

    var marker = L.marker(e.latlng).addTo(map);//create marker at clicked location
    markers.push(marker);//add to markers
    selectedPoints.push(e.latlng);//store clicked location

    if (selectedPoints.length === 2) {
        document.getElementById('confirm-path').disabled = false;//check if two points are put
    }
}

map.on('click', onMapClick);// event listener for clicking on map to put markers

function onConfirmClick(e){
    if (selectedPoints.length < 2) {
        alert("Please select two points on the map.");
        return;
    }
    //get clicked coords
    var startLat = selectedPoints[0].lat;
    var startLon = selectedPoints[0].lng;
    var goalLat = selectedPoints[1].lat;
    var goalLon = selectedPoints[1].lng;

    // get nearest nodes for start and goal
    fetch(`/pathfinding/get_nearest_node?lat=${startLat}&lon=${startLon}`)
        .then(response => response.json())
        .then(data1 => {
            let startNode = data1.node_id;

            return fetch(`/pathfinding/get_nearest_node?lat=${goalLat}&lon=${goalLon}`)
                .then(response => response.json())
                .then(data2 => {
                    let goalNode = data2.node_id;

                    // redirect to map_template.html with the selected nodes to display computed route
                    window.location.href = `/pathfinding/visualize_path?start=${startNode}&goal=${goalNode}`;
                });
        })
        .catch(error => console.error('Error:', error));
}

document.getElementById('confirm-path').addEventListener('click', onConfirmClick);//event listener for confirming path to check points
