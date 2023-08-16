class Coordinate {
    static toCoordinate(o, columns) {
        return new Coordinate(o.r, o.c, columns);
    }

    c;//x
    r;//y
    i;//index in single dimension array

    constructor(r, c, columns) {
        this.r = r;
        this.c = c;
        this.setOrder(columns);
    }

    toString() {
        return `(${this.r},${this.c})`;
    }

    equals(p) {
        return this.r == p.r && this.c == p.c;
    }

    setOrder(columns) {
        this.i = this.r * columns + this.c;
    }

    compare(p) {
        return this.i < p.i ? -1 : this.i > p.i ? 1 : 0;
    }
}

class CoordinateArray extends Array {
    sortCoordinates() {
        this.sort((a, b) => a.i - b.i);
    }

    containsCoordinate(coordinate) {
        return this.positionOfCoordinate(coordinate) != -1;
    }

    positionOfCoordinate(coordinate) {
        for (let i = 0; i < this.length; i++) if (this[i].equals(coordinate)) return i;
        return -1;
    }

    findCoordinate(coordinate) {
        return this[this.positionOfCoordinate(coordinate)];
    }

    removeCoordinate(coordinate) {
        const index = this.indexOf(this.findCoordinate(coordinate));
        if (index > -1) this.splice(index, 1);
    }

    addCoordinate(r, c, columns) {
        this.push(new Coordinate(r, c, columns));
    }
}

// config is read from URL (see below)
const urlParams = new URLSearchParams(window.location.search);

let delay = 200; // delay between steps in ms in auto solve mode
let random = 25; // number of random obstacles
const updateGridVisually = true; // set to false to disable visualisation of the grid
const defRows = 10; // default number of rows
const defCols = 10; // default number of columns
const L = 10; // unit for distance of neighbours in a straight line: ( 1 ) * 10
const LD = 14; // diagonal: √( L^2 + L^2 ) * 10 [rounded to 0 decimal places]
class AStarMazeWalker {
    closed = new CoordinateArray();
    open = new CoordinateArray();
    current = null;
    neighbours = new CoordinateArray();
    neighboursOpen = new CoordinateArray();
    obstacles = new CoordinateArray();
    columns = defCols; // number of columns
    rows = defRows; // number of rows
    cutCorners = false; // allow diagonal movement through corners
    goal = null; // goal cell
    start = null; // start cell
    horizontalWalls = new CoordinateArray(); // blocked by wall to the bottom
    verticalWalls = new CoordinateArray(); // blocked by wall to the right
    calculationIterations = 0; // total calculation iterations
    solvedPath = false; // path from start to goal

    // copy setup from another data object
    copyFrom(data) {
        this.clear();
        data.obstacles.forEach(o => this.obstacles.push(Coordinate.toCoordinate(o, data.columns)));
        this.columns = data.columns;
        this.rows = data.rows;
        this.cutCorners = data.cutCorners;
        this.goal = Coordinate.toCoordinate(data.goal, data.columns);
        this.start = Coordinate.toCoordinate(data.start, data.columns);
        data.horizontalWalls.forEach(o => this.horizontalWalls.push(Coordinate.toCoordinate(o, data.columns)));
        data.verticalWalls.forEach(o => this.verticalWalls.push(Coordinate.toCoordinate(o, data.columns)));
    }

    // clear all data
    clear() {
        this.reset();
        this.obstacles = new CoordinateArray();
        this.goal = null;
        this.start = null;
        this.horizontalWalls = new CoordinateArray();
        this.verticalWalls = new CoordinateArray();
    }

    // reset to initial state
    reset() {
        this.closed = new CoordinateArray();
        this.open = new CoordinateArray();
        this.current = null;
        this.neighbours = new CoordinateArray();
        this.neighboursOpen = new CoordinateArray();
        this.calculationIterations = 0;
        this.solvedPath = false;
    }

    setupData() {
        this.start.g = 0; // distance from start to start is 0 (obviously): number is used by first calculation step
        this.current = this.start; // start cell should be added as current for first calculation step
        // this.obstacles.setCoordinatesOrder(this.columns); // calculate index for faster findCoordinate
        this.obstacles.sortCoordinates(); // sort obstacles for faster findCoordinate
    }

    // all neighbours of a point including diagonals even if they are blocked or out of bounds (for the purpose of checking if a point is a neighbour of another point)
    allNeighboursOf(coordinate) {
        let neighbours = new CoordinateArray();
        neighbours.addCoordinate(coordinate.r - 1, coordinate.c - 1, this.columns);
        neighbours.addCoordinate(coordinate.r - 1, coordinate.c, this.columns);
        neighbours.addCoordinate(coordinate.r - 1, coordinate.c + 1, this.columns);
        neighbours.addCoordinate(coordinate.r, coordinate.c - 1, this.columns);
        neighbours.addCoordinate(coordinate.r, coordinate.c + 1, this.columns);
        neighbours.addCoordinate(coordinate.r + 1, coordinate.c - 1, this.columns);
        neighbours.addCoordinate(coordinate.r + 1, coordinate.c, this.columns);
        neighbours.addCoordinate(coordinate.r + 1, coordinate.c + 1, this.columns);
        return neighbours;
    }

    insideBounds(n) {
        return (0 <= n.r && n.r < this.rows) && (0 <= n.c && n.c < this.columns);
    }

    // all neighbours of a point that are not blocked or out of bounds, and that are not already closed (handled)
    possibleNeighboursOf(coordinate) {
        let neighbours = this.allNeighboursOf(coordinate);
        // filter out out of bounds
        neighbours = neighbours.filter(n => this.insideBounds(n));
        // filter out those already closed (handled)
        neighbours = neighbours.filter(n => !this.closed.containsCoordinate(n));
        // filter out those blocked by obstacles
        neighbours = neighbours.filter(n => !this.isBlockedByObstacle(coordinate, n));
        // filter out those blocked by walls
        neighbours = neighbours.filter(n => !this.isBlockedByWall(coordinate, n));
        return neighbours;
    }

    // update direction label (arrow)
    getCoordinateLabel(parentCoordinate, childCoordinate) {
        if (parentCoordinate.r == childCoordinate.r) return parentCoordinate.c > childCoordinate.c ? '→' : '←';
        if (parentCoordinate.c == childCoordinate.c) return parentCoordinate.r > childCoordinate.r ? '↓' : '↑';
        if (parentCoordinate.r > childCoordinate.r) return parentCoordinate.c > childCoordinate.c ? '↘' : '↙';
        if (parentCoordinate.r < childCoordinate.r) return parentCoordinate.c > childCoordinate.c ? '↗' : '↖';
        return null;
    }

    isBlockedByObstacle(fromCoordinate, toCoordinate) {
        // targetCoordinate is obstacle: blocked
        if (this.obstacles.containsCoordinate(toCoordinate)) return true;

        // straight movement never blocked by obstacles
        if (fromCoordinate.r == toCoordinate.r || fromCoordinate.c == toCoordinate.c) return false;

        // other neighbours in square
        const coordinate1 = new Coordinate(fromCoordinate.r, toCoordinate.c, this.colums);
        const coordinate2 = new Coordinate(toCoordinate.r, fromCoordinate.c, this.colums);

        // filter out movement between obstacles if cutCorners
        // n X
        // X n
        // 
        // X n
        // n X

        // filter out movement close to corners if not cutCorners
        // n X
        // . n
        // 
        // X n
        // n .

        return this.cutCorners ? (this.obstacles.containsCoordinate(coordinate1) && this.obstacles.containsCoordinate(coordinate2)) : (this.obstacles.containsCoordinate(coordinate1) || this.obstacles.containsCoordinate(coordinate2));
    }

    isBlockedByWall(fromCoordinate, toCoordinate) {
        if (fromCoordinate.c == toCoordinate.c) { // vertical up or down
            if (fromCoordinate.r + 1 == toCoordinate.r) {
                // fromCoordinate
                // ↓
                // toCoordinate
                if (this.horizontalWalls.containsCoordinate(fromCoordinate)) {
                    return true;
                }
            } else /* if (fromCoordinate.r == toCoordinate.r + 1) */ {
                // toCoordinate
                // ↑
                // fromCoordinate 
                if (this.horizontalWalls.containsCoordinate(toCoordinate)) {
                    return true;
                }
            }
        } else if (fromCoordinate.r == toCoordinate.r) {      // move left or right
            if (fromCoordinate.c + 1 == toCoordinate.c) {
                // fromCoordinate → toCoordinate
                if (this.verticalWalls.containsCoordinate(fromCoordinate)) {
                    return true;
                }
            } else /* if (fromCoordinate.c == toCoordinate.c + 1) */ {
                //  toCoordinate ← fromCoordinate
                if (this.verticalWalls.containsCoordinate(toCoordinate)) {
                    return true;
                }
            }
        } else /* if (fromCoordinate.r != toCoordinate.r && fromCoordinate.c != toCoordinate.c) */ { // diagonal move

            const topLeftPoint = new Coordinate(Math.min(fromCoordinate.r, toCoordinate.r), Math.min(fromCoordinate.c, toCoordinate.c), this.colums);
            const topRightPoint = new Coordinate(topLeftPoint.r, topLeftPoint.c + 1, this.colums);
            const bottomLeftPoint = new Coordinate(topLeftPoint.r + 1, topLeftPoint.c, this.colums);
            if (this.cutCorners) {
                if (this.horizontalWalls.containsCoordinate(topLeftPoint) && this.horizontalWalls.containsCoordinate(topRightPoint)) {
                    return true;
                }
                if (this.verticalWalls.containsCoordinate(topLeftPoint) && this.verticalWalls.containsCoordinate(bottomLeftPoint)) {
                    return true;
                }
            } else {
                if (this.horizontalWalls.containsCoordinate(topLeftPoint)) {
                    return true;
                }
                if (this.verticalWalls.containsCoordinate(topLeftPoint)) {
                    return true;
                }
                if (this.horizontalWalls.containsCoordinate(topRightPoint)) {
                    return true;
                }
                if (this.verticalWalls.containsCoordinate(bottomLeftPoint)) {
                    return true;
                }
            }
        }

        return false;
    }

    // Euclidean distance between points
    euclideanDistance(coordinate1, coordinate2) {
        //  d = √[(x2 – x1)^2 + (y2 – y1)^2]
        return Math.sqrt(Math.pow(coordinate1.r - coordinate2.r, 2) + Math.pow(coordinate1.c - coordinate2.c, 2));
    }

    // Manhattan distance: distance in horizontal and vertical steps taken to goal
    manhattanDistance(coordinate1, coordinate2) {
        return Math.abs(coordinate1.r - coordinate2.r) + Math.abs(coordinate1.c - coordinate2.c);
    }

    // heuristic: Manhattan distance multiplied by 10
    h(coordinate1, coordinate2) {
        // Euclidean needs more calculations than Manhattan, it's also slower, but it's more accurate
        // return Math.floor(this.euclideanDistance(point1, point2) * 10);
        return this.manhattanDistance(coordinate1, coordinate2) * 10;
    }

    // Euclidean distance: distance for adjacent cells, multiplied by 10, either L (1 * 10) or LD (1.4… * 10)
    g(coordinate1, coordinate2) {
        return (coordinate1.r == coordinate2.r || coordinate1.c == coordinate2.c) ? L : LD;
    }

    // solve the maze and return the solved path or null if no path was found, max 1000 steps (default)
    solve(maxSteps = 1000) {
        this.reset();
        this.setupData();
        while (maxSteps-- > 0 && !this.foundGoal && !this.noPath) {
            this.step();
        }
        return this.path;
    }

    // take a single calculation iteration to solve the maze
    step() {
        if (this.noPath || this.foundGoal) return;

        this.calculationIterations++;
        this.closed.push(this.current);

        if (updateGridVisually) {
            document.getElementById('stepsTaken').innerHTML = this.calculationIterations;
            if (this.start != this.current) {
                const domNode = this.domNode(this.current);
                domNode.classList.add('closed');
                domNode.querySelector('.order').innerHTML = this.closed.length - 1;
            }
        }

        this.neighbours = this.possibleNeighboursOf(this.current);

        this.neighbours.filter(neighbour => !this.open.containsCoordinate(neighbour)).forEach(neighbour => {
            this.open.push(neighbour);
            neighbour.h = this.h(neighbour, this.goal);
            neighbour.g = this.current.g + this.g(neighbour, this.current);
            neighbour.f = neighbour.h + neighbour.g;
            neighbour.parent = this.current;
            if (updateGridVisually) if (neighbour.h > 0) {
                const domNode = this.domNode(neighbour);
                domNode.classList.add('open');
                neighbour.label = this.getCoordinateLabel(this.current, neighbour);
                domNode.innerHTML = "<div><div class='light coord'>(r:" + neighbour.r + ",c:" + neighbour.c + ")</div><div class='light h'>h=" + neighbour.h + "</div><div class='light g'>g=" + neighbour.g + "</div><div class='dark f'>f=" + neighbour.f + "</div><div class='parent'>" + neighbour.label + "</div><div class='dark order'>&nbsp;</div></div>";
            }
        });

        this.neighbours.filter(neighbour => this.open.containsCoordinate(neighbour)).map(neighbour => this.open.findCoordinate(neighbour)).forEach(neighbour => {
            // recalculate if shorter path when going through current
            const g = this.current.g + this.g(neighbour, this.current);
            if (g < neighbour.g) { // only recalculate if g is shorter
                // h doesn't change /* point.h = this.h(point, this.goal); */
                neighbour.g = g; // g is calculate above
                neighbour.f = neighbour.h + neighbour.g; // recalculate
                neighbour.parent = this.current; // change parent
                if (updateGridVisually) if (neighbour.h > 0) {
                    const domNode = this.domNode(neighbour);
                    domNode.classList.add('adjusted');
                    neighbour.label = this.getCoordinateLabel(this.current, neighbour);
                    domNode.querySelector('.g').innerHTML = "g=" + neighbour.g;
                    domNode.querySelector('.f').innerHTML = "f=" + neighbour.f;
                    domNode.querySelector('.parent').innerHTML = neighbour.label;
                }
            }
        });

        if (this.open.length == 0) {
            this.noPath = true;
            console.log('no path');
            if (updateGridVisually) this.domNode(this.goal).style.backgroundColor = "purple";
            return;
        }

        this.open.sort((a, b) => a.f - b.f || a.h - b.h);
        if (updateGridVisually && this.current) this.domNode(this.current).classList.remove('current');
        this.current = this.open.shift();
        if (this.goal.equals(this.current)) {
            this.foundGoal = true;
            this.goal.parent = this.current;
            this.solvedPath = this.buildPath();
            this.debugPath();
            return;
        } else {
            if (updateGridVisually) this.domNode(this.current).classList.add('current');
        }
    }

    domNode(coordinate) {
        return document.getElementById(`${coordinate.r}-${coordinate.c}`);
    }

    buildPath() {
        if (!this.foundGoal) return;
        let path = new CoordinateArray();
        let pathCoordinate = this.goal;
        while (pathCoordinate != this.start) {
            path.push(pathCoordinate);
            pathCoordinate = pathCoordinate.parent;
        }
        path.push(this.start);
        path.reverse();
        return path.splice(1, path.length - 2); // TODO why is goal twice included?
    }

    debugPath() {
        if (!this.foundGoal) return;
        if (!this.solvedPath) buildPath();
        if (updateGridVisually) this.solvedPath.forEach((coordinate, index) => {
            const domNode = document.getElementById(`${coordinate.r}-${coordinate.c}`);
            domNode.classList.add('path');
            const orderSpan = domNode.querySelector('.order');
            if (orderSpan) orderSpan.innerHTML = '#' + (index + 1);
        });
        this.solvedPath.forEach((coordinate, index) => console.log(index + ": " + coordinate.r + "," + coordinate.c));
    }
}

const mazeWalker = new AStarMazeWalker();

{
    if (urlParams.get('data')) mazeWalker.copyFrom(JSON.parse(decodeURIComponent(urlParams.get('data'))));
    // separate URL parameters take precedence over data parameter
    if (urlParams.get('cutCorners')) mazeWalker.cutCorners = urlParams.get('cutCorners') == 'true';
    if (urlParams.get('columns')) mazeWalker.columns = parseInt(urlParams.get('columns'));
    if (urlParams.get('rows')) mazeWalker.rows = parseInt(urlParams.get('rows'));
    if (urlParams.get('delay')) delay = parseInt(urlParams.get('delay'));
    if (urlParams.get('rnd')) random = parseInt(urlParams.get('rnd'));
}

if (!urlParams.get('data')) {

    if (urlParams.get('rnd')) {

        console.log('random data');
        let cellOptions = new CoordinateArray();
        for (let ri = 0; ri < mazeWalker.rows; ri++) {
            for (let ci = 0; ci < mazeWalker.columns; ci++) {
                cellOptions.addCoordinate(ri, ci, mazeWalker.columns);
            }
        }


        let boundRows = Math.ceil(mazeWalker.rows / 4);
        let boundColumns = Math.ceil(mazeWalker.columns / 4);

        mazeWalker.start = new Coordinate(Math.floor(Math.random() * boundRows), Math.floor(Math.random() * boundColumns, mazeWalker.colums));
        cellOptions.removeCoordinate(mazeWalker.start);

        mazeWalker.goal = new Coordinate(mazeWalker.rows - 1 - Math.floor(Math.random() * boundRows), mazeWalker.columns - 1 - Math.floor(Math.random() * boundColumns), mazeWalker.colums);
        cellOptions.removeCoordinate(mazeWalker.goal);

        let cellPicked;
        let j;
        let i;
        for (; i < random; i++) {
            j = Math.floor(Math.random() * cellOptions.length);
            cellPicked = cellOptions[j];
            mazeWalker.obstacles.push(cellPicked);
            cellOptions.removeCoordinate(cellPicked);
        }

    } else {

        console.log('example data');
        // example 1
        if (false) {
            mazeWalker.start = new Coordinate(2, 2, mazeWalker.colums);
            mazeWalker.goal = new Coordinate(3, 6, mazeWalker.colums);
            mazeWalker.obstacles.addCoordinate(3, 1, mazeWalker.colums);
            mazeWalker.obstacles.addCoordinate(3, 2, mazeWalker.colums);
            mazeWalker.obstacles.addCoordinate(3, 3, mazeWalker.colums);
            mazeWalker.obstacles.addCoordinate(3, 4, mazeWalker.colums);
            mazeWalker.obstacles.addCoordinate(2, 4, mazeWalker.colums);
            mazeWalker.obstacles.addCoordinate(1, 4, mazeWalker.colums);
            if (false) {
                mazeWalker.horizontalWalls.addCoordinate(1, 5, mazeWalker.colums);
                mazeWalker.horizontalWalls.addCoordinate(1, 6, mazeWalker.colums);
            }
        }
        // example 2
        else if (false) {
            mazeWalker.start = new Coordinate(1, 1, mazeWalker.colums);
            mazeWalker.goal = new Coordinate(1, 3, mazeWalker.colums);
            mazeWalker.obstacles.addCoordinate(1, 2, mazeWalker.colums);
        }
        // example 3
        else if (true) {
            const obstacles = [{ "r": 5, "c": 5 }, { "r": 6, "c": 9 }, { "r": 0, "c": 2 }, { "r": 6, "c": 8 }, { "r": 6, "c": 6 }, { "r": 3, "c": 8 }, { "r": 9, "c": 2 }, { "r": 1, "c": 1 }, { "r": 4, "c": 4 }, { "r": 3, "c": 5 }, { "r": 6, "c": 4 }, { "r": 3, "c": 9 }, { "r": 2, "c": 9 }, { "r": 0, "c": 0 }, { "r": 1, "c": 0 }, { "r": 2, "c": 7 }, { "r": 8, "c": 5 }, { "r": 0, "c": 5 }, { "r": 8, "c": 7 }, { "r": 1, "c": 2 }, { "r": 6, "c": 3 }, { "r": 9, "c": 0 }, { "r": 6, "c": 7 }, { "r": 8, "c": 6 }, { "r": 3, "c": 0 }];
            obstacles.forEach(o => mazeWalker.obstacles.push(Coordinate.toCoordinate(o, mazeWalker.colums)));
            mazeWalker.start = new Coordinate(9, 1, mazeWalker.colums);
            mazeWalker.goal = new Coordinate(4, 5, mazeWalker.colums);
        }
    }

}

{
    document.getElementById('cutting').innerHTML = mazeWalker.cutCorners ? 'yes' : 'no';
    document.getElementById('save').href = "?data=" + encodeURIComponent(JSON.stringify(mazeWalker));
    document.getElementById('toggle').href = "?cutCorners=" + !mazeWalker.cutCorners + "&data=" + encodeURIComponent(JSON.stringify(mazeWalker));
    document.getElementById('rnd').href = "?columns=" + mazeWalker.columns + "&rows=" + mazeWalker.rows + "&rnd=" + random + "&cutCorners=" + mazeWalker.cutCorners + "&delay=" + delay;
}

mazeWalker.setupData();

if (updateGridVisually) {
    const grid = document.getElementById("grid");
    let innerHTML = "";
    for (let row = 0; row < mazeWalker.rows; row++) {
        innerHTML += `<div class="row" id="${row}">`;
        for (let column = 0; column < mazeWalker.columns; column++) {
            const coordinate = new Coordinate(row, column, mazeWalker.colums);
            let classList = "cell";
            let content = "";
            if (mazeWalker.start.equals(coordinate)) {
                classList += " start";
                content = "S";
            } else if (mazeWalker.goal.equals(coordinate)) {
                classList += " goal";
                content = "G";
            } else if (mazeWalker.obstacles.containsCoordinate(coordinate)) {
                classList += " blocked";
                content = "X";
            }
            if (mazeWalker.verticalWalls.containsCoordinate(coordinate)) {
                classList += " wall-vertical";
            }
            const coordinateLeft = new Coordinate(row, column - 1, mazeWalker.colums);
            if (mazeWalker.verticalWalls.containsCoordinate(coordinateLeft)) {
                classList += " wall-vertical-opposite";
            }
            if (mazeWalker.horizontalWalls.containsCoordinate(coordinate)) {
                classList += " wall-horizontal";
            }
            const coordinateAbove = new Coordinate(row - 1, column, mazeWalker.colums);
            if (mazeWalker.horizontalWalls.containsCoordinate(coordinateAbove)) {
                classList += " wall-horizontal-opposite";
            }
            innerHTML += `<div class="${classList}" id="${row}-${column}"><span>${content}(r:${row},c:${column})</span></div>`;
        }
        innerHTML += "</div>";
    }
    grid.innerHTML += innerHTML;
}

document.getElementById('next').addEventListener("click", event => {
    event.preventDefault();
    mazeWalker.step();
});

document.getElementById('auto').addEventListener("click", event => {
    event.preventDefault();
    const interval = setInterval(() => {
        if (mazeWalker.foundGoal || mazeWalker.noPath) {
            clearInterval(interval);
            return;
        }
        mazeWalker.step();
    }, delay);
});