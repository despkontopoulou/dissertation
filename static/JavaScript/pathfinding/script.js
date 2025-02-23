function findPath() {
    let start = document.getElementById("start").value;
    let goal = document.getElementById("goal").value;

    fetch(`/pathfinding/find_path?start=${start}&goal=${goal}`)
        .then(response => response.json())
        .then(data => {
            if (data.error) {
                document.getElementById("result").innerHTML = `<p style="color:red;">${data.error}</p>`;
            } else {
                document.getElementById("result").innerHTML = `
                    <h3>Results:</h3>
                    <strong>Dijkstra:</strong> Path = ${data.dijkstra.path}, Time = ${data.dijkstra.time.toFixed(5)}s <br>
                    <strong>A*:</strong> Path = ${data.astar.path}, Time = ${data.astar.time.toFixed(5)}s
                `;
            }
        })
        .catch(error => console.error("Error fetching path:", error));
}
