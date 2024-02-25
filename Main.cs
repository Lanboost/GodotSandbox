using DeBroglie.Models;
using DeBroglie.Topo;
using DeBroglie;
using Godot;
using System;
using System.Collections.Generic;
using DeBroglie.Rot;
using Lanboost.PathFinding.Astar;
using Lanboost.PathFinding.Graph;
using System.Collections;
using System.Drawing;

public enum ENineCollisionFlag
{
    None = 0,
    Full = 511,

    Middle = 1,
    North = 2,
    NorthEast = 4,
    East = 8,
    SouthEast = 16,
    South = 32,
    SouthWest = 64,
    West = 128,
    NorthWest = 256
}

public interface ICollisionWorld<T>
{
    public int ChunkSize();

    public float TileScale();
    public IEnumerable<T> GetChunk(int x, int y);

    public IEnumerable<ulong> GetAvailableChunks();
}

public interface INineCollisionWorld: ICollisionWorld<int>
{
}

public interface IBaseCollisionWorld : ICollisionWorld<bool>
{
}

public class SubdivideCollisionWorld : IBaseCollisionWorld
{
    bool[] rowFlags;
    IBaseCollisionWorld world;
    int divisions;

    public SubdivideCollisionWorld(IBaseCollisionWorld world, int divisions)
    {
        this.world = world;
        this.divisions = divisions;
        rowFlags = new bool[world.ChunkSize()];
    }

    public int ChunkSize()
    {
        return world.ChunkSize()* divisions;
    }

    IEnumerable<bool> ExpandTileFlagRowToTileRows(bool[] flags)
    {
        for (int row = 0; row < 3; row++)
        {
            for (int ix = 0; ix < flags.Length; ix++)
            {
                for (int i = 0; i < 3; i++)
                {
                    yield return flags[ix];
                }
            }
        }
    }


    public IEnumerable<bool> GetChunk(int x, int y)
    {
        int cx = 0;
        int cy = 0;
        // We need to expand each cell into a 3x3, so store an entire row, then output 3 rows back with 3 times the cells
        foreach (var blocked in world.GetChunk(x, y))
        {
            rowFlags[cx] = blocked;
            cx++;
            if (cx == world.ChunkSize())
            {
                foreach (var temp in ExpandTileFlagRowToTileRows(rowFlags))
                {
                    yield return temp;
                }

                cx = 0;
                cy++;
            }
        }
    }

    public IEnumerable<ulong> GetAvailableChunks()
    {
        foreach (var key in world.GetAvailableChunks())
        {
            yield return key;
        }
    }

    public float TileScale()
    {
        return world.TileScale() * 1/3f;
    }
}


public class ExpandedTileCollisionWorld : IBaseCollisionWorld
{
    IBaseCollisionWorld world;
    bool[][] tempChunk;
    bool[] tempRow;

    public ExpandedTileCollisionWorld(IBaseCollisionWorld world)
    {
        this.world = world;

        // Create temp arrays of size 2 bigger (to be able to store surrounding chunk block data)
        tempChunk = new bool[world.ChunkSize()+2][];
        tempRow = new bool[world.ChunkSize()+2];
        for (int iy = 0; iy < tempChunk.Length; iy++)
        {
            tempChunk[iy] = new bool[world.ChunkSize()+2];
        }
    }

    public int ChunkSize()
    {
        return world.ChunkSize();
    }

    void LoadMainChunkIntoTemp(int x, int y)
    {
        int cx = 0;
        int cy = 0;
        // Copy over from underlying world, with 1,1 in offset (so we can get blocked from surrounding chunks)
        foreach (var blocked in world.GetChunk(x, y))
        {
            tempChunk[cy + 1][cx + 1] = blocked;
            cx++;
            if (cx == this.ChunkSize())
            {
                cx = 0;
                cy++;
            }
        }
    }

    void LoadSurroundingChunksIntoTemp(int x, int y)
    {
        
        // Copy over surrounding chunks from underlying world
        for (int iy = -1; iy <= 1; iy++)
        {
            for (int ix = -1; ix <= 1; ix++)
            {
                if (ix == 0 && iy == 0)
                {
                    continue;
                }

                int cx = 0;
                int cy = 0;
                foreach (var blocked in world.GetChunk(x + ix, y + iy))
                {
                    int nx = -1;
                    int ny = -1;
                    if (ix == -1)
                    {
                        if (cx == this.ChunkSize() - 1)
                        {
                            nx = 0;
                        }
                    }
                    else if (ix == 1)
                    {
                        if (cx == 0)
                        {
                            nx = this.ChunkSize() + 1;
                        }
                    }
                    else
                    {
                        nx = cx + 1;
                    }

                    if (iy == -1)
                    {
                        if (cy == this.ChunkSize() - 1)
                        {
                            ny = 0;
                        }
                    }
                    else if (iy == 1)
                    {
                        if (cy == 0)
                        {
                            ny = this.ChunkSize() + 1;
                        }
                    }
                    else
                    {
                        ny = cy + 1;
                    }
                    if (nx >= 0 && ny >= 0)
                    {
                        tempChunk[ny][nx] = blocked;
                    }

                    cx++;
                    if (cx == this.ChunkSize())
                    {
                        cx = 0;
                        cy++;
                    }
                }

            }
        }
    }


    public IEnumerable<bool> GetChunk(int x, int y)
    {
        LoadMainChunkIntoTemp(x, y);
        LoadSurroundingChunksIntoTemp(x, y);

        var size = tempChunk.Length;

        // Expand in X
        for (int outer = 0; outer < size; outer++)
        {
            // First copy over row
            for (int inner = 0; inner < size; inner++)
            {
                tempRow[inner] = tempChunk[outer][inner];
            }

            for (int inner = 0; inner < size; inner++)
            {
                var blocked = false;
                for (int i = -1; i <= 1; i++)
                {
                    var ix = inner + i;
                    if(ix >= 0 && ix < size)
                    {
                        blocked = blocked || tempRow[ix];
                    }
                }
                tempChunk[outer][inner] = blocked;
            }
        }

        // Expand in y (just swapped sx, sy in array
        for (int outer = 0; outer < size; outer++)
        {
            // First copy over row
            for (int inner = 0; inner < size; inner++)
            {
                tempRow[inner] = tempChunk[inner][outer];
            }

            for (int inner = 0; inner < size; inner++)
            {
                var blocked = false;
                for (int i = -1; i <= 1; i++)
                {
                    var ix = inner + i;
                    if (ix >= 0 && ix < size)
                    {
                        blocked = blocked || tempRow[ix];
                    }
                }
                tempChunk[inner][outer] = blocked;
            }
        }

        // Return "main" chunk
        for (int iy = 0; iy < this.ChunkSize(); iy++)
        {
            for (int ix = 0; ix < this.ChunkSize(); ix++)
            {
                yield return tempChunk[iy + 1][ix + 1];
            }
        }
    }

    public IEnumerable<ulong> GetAvailableChunks()
    {
        foreach(var key in world.GetAvailableChunks())
        {
            yield return key;
        }
    }

    public float TileScale()
    {
        return world.TileScale();
    }
}

public class NineBlockCollisionWorld : IBaseCollisionWorld
{
    INineCollisionWorld world;
    int[] rowFlags;

    static int[][] blockLookup = new int[][]
    {
        new int[]
        {
            (int) ENineCollisionFlag.NorthWest,
            (int) ENineCollisionFlag.North,
            (int) ENineCollisionFlag.NorthEast
        },
        new int[]
        {
            (int) ENineCollisionFlag.West,
            (int) ENineCollisionFlag.Middle,
            (int) ENineCollisionFlag.East
        },
        new int[]
        {
            (int) ENineCollisionFlag.SouthWest,
            (int) ENineCollisionFlag.South,
            (int) ENineCollisionFlag.SouthEast
        }
    };

    public NineBlockCollisionWorld(INineCollisionWorld world)
    {
        this.world = world;
        rowFlags = new int[world.ChunkSize()];
    }

    public int ChunkSize()
    {
        return world.ChunkSize()*3;
    }


    IEnumerable<bool> TileToBlockedRow(int flag, int row)
    {
        for(int i=0; i<3; i++)
        {
            yield return (flag & blockLookup[row][i]) > 0;
        }
    }

    IEnumerable<bool> ExpandTileFlagRowToTileRows(int[] flags)
    {
        for (int row = 0; row < 3; row++)
        {
            for (int ix = 0; ix < flags.Length; ix++)
            {
                foreach (var blocked in TileToBlockedRow(flags[ix], row))
                {
                    yield return blocked;
                }
            }
        }
    }


    public IEnumerable<bool> GetChunk(int x, int y)
    {
        int cx = 0;
        int cy = 0;
        // We need to expand each cell into a 3x3, so store an entire row, then output 3 rows back with 3 times the cells
        foreach (var blockFlag in world.GetChunk(x, y))
        {
            rowFlags[cx] = blockFlag;
            cx++;
            if (cx == world.ChunkSize())
            {
                foreach(var blocked in ExpandTileFlagRowToTileRows(rowFlags))
                {
                    yield return blocked;
                }

                cx = 0;
                cy++;
            }
        }
    }

    public IEnumerable<ulong> GetAvailableChunks()
    {
        foreach (var key in world.GetAvailableChunks())
        {
            yield return key;
        }
    }

    public float TileScale()
    {
        return world.TileScale() * 1 / 3f;
    }
}



public class WaveFunctionCollapseWorld: IBaseCollisionWorld
{
    int chunkSize = 8;
    public Dictionary<ulong, bool[][]> chunks = new Dictionary<ulong, bool[][]> ();

    public WaveFunctionCollapseWorld(int chunkSize)
    {
        this.chunkSize = chunkSize;
    }

    public void GenerateChunk(int x, int y)
    {
        ITopoArray<char> sample = TopoArray.Create(new[]
        {
            //https://github.com/mxgmn/WaveFunctionCollapse
            //0 water, 1 grass, 2 mountain, 3 sand
            new[] { '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0'},
            new[] { '0', '0', '0', '1', '0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '0', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '0', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '1', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '1', '1'},
            new[] { '0', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '0', '0', '1', '0', '0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
            new[] { '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0'},
            new[] { '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0'},
            new[] { '0', '0', '1', '1', '1', '1', '1', '1', '0', '0', '1', '1', '1', '1', '0', '0'},
            new[] { '1', '1', '1', '1', '1', '1', '1', '1', '0', '0', '1', '1', '1', '1', '1', '1'},
            new[] { '0', '0', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '0', '0'},
            new[] { '0', '0', '1', '1', '1', '1', '1', '1', '0', '0', '1', '1', '1', '1', '0', '0'},
            new[] { '0', '0', '1', '1', '1', '1', '1', '1', '0', '0', '0', '0', '1', '0', '0', '0'},
            new[] { '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0'}
        }, periodic: false);
        // Specify the model used for generation
        var model = new OverlappingModel(sample.ToTiles(), 3, 4, true);
        var msize = this.ChunkSize();
        // Set the output dimensions
        var topology = new GridTopology(msize, msize, periodic: false);

        var options = new TilePropagatorOptions();
        var random = new Random(y*1000+x);

        options.RandomDouble = () => { return random.NextDouble(); };
        // Acturally run the algorithm
        var propagator = new TilePropagator(model, topology, options);

        var status = propagator.Run();
        if (status != Resolution.Decided) throw new Exception("Undecided");
        var output = propagator.ToValueArray<char>();
        
        var data = new bool[msize][];
        for (var iy = 0; iy < msize; iy++)
        {
            data[iy] = new bool[msize];
            for (var ix = 0; ix < msize; ix++)
            {
                if (output.Get(ix, iy) == '0')
                {
                    data[iy][ix] = true;
                }
                else
                {
                    data[iy][ix] = false;
                }
            }
        }
        GD.Print(x, y, PositionKey.Key(x, y));
        this.chunks.Add(PositionKey.Key(x, y), data);
    }

    public int ChunkSize()
    {
        return chunkSize;
    }

    public IEnumerable<bool> GetChunk(int x, int y)
    {
        var key = PositionKey.Key(x, y);
        if (key == ulong.MaxValue || !chunks.ContainsKey(key))
        {
            for (var iy = 0; iy < ChunkSize(); iy++)
            {
                for (var ix = 0; ix < ChunkSize(); ix++)
                {
                    yield return true;
                }
            }
        }
        else
        {
            if (chunks.ContainsKey(key))
            {
                var data = chunks[key];
                for (var iy = 0; iy < data.Length; iy++)
                {
                    for (var ix = 0; ix < data[iy].Length; ix++)
                    {
                        yield return data[iy][ix];
                    }
                }
            }
        }
    }

    public IEnumerable<ulong> GetAvailableChunks()
    {
        foreach(var key in chunks.Keys)
        {
            yield return key;
        }
    }

    public float TileScale()
    {
        return 1;
    }
}



public static class MyExtensions
{
    public static IEnumerable<(int, int, T)> GetChunkSimple<T>(this ICollisionWorld<T> world, int x, int y)
    {
        int cx = 0;
        int cy = 0;
        // Copy over from underlying world, with 1,1 in offset (so we can get blocked from surrounding chunks)
        foreach (var value in world.GetChunk(x, y))
        {
            yield return (cx, cy, value);
            cx++;
            if (cx == world.ChunkSize())
            {
                cx = 0;
                cy++;
            }
        }
    }
}
public delegate void PropertyChangeEvent(object sender);
public class WorldData
{
    public object baseWorld;

    public List<object> worldCollisionSteps = new List<object>();

    public WorldData(object baseWorld)
    {
        this.baseWorld = baseWorld;
    }

    

    protected int _DisplayedCollisionLayer = -1;
    #region property change event
    public int DisplayedCollisionLayer
    {
        get => _DisplayedCollisionLayer;
        set
        {
            if (_DisplayedCollisionLayer == value)
            {
                return;
            }
            _DisplayedCollisionLayer = value;
            DisplayedCollisionLayerOnChanged?.Invoke(this);
        }
    }
    public event PropertyChangeEvent DisplayedCollisionLayerOnChanged;
    #endregion


    protected bool _DisplayCollider;
    #region property change event
    public bool DisplayCollider
    {
        get => _DisplayCollider;
        set
        {
            if (_DisplayCollider == value)
            {
                return;
            }
            _DisplayCollider = value;
            DisplayColliderOnChanged?.Invoke(this);
        }
    }
    public event PropertyChangeEvent DisplayColliderOnChanged;
    #endregion




}

public partial class Main : Node2D
{
    float[][] HeightData;

    
    public NavMesh navMesh;



    protected WorldData _World;
    #region property change event
    public WorldData World
    {
        get => _World;
        set
        {
            if (_World == value)
            {
                return;
            }
            _World = value;
            WorldOnChanged?.Invoke(this);
        }
    }
    public event PropertyChangeEvent WorldOnChanged;
    #endregion



    protected bool _NavMeshUpdated;
    #region property change event
    public bool NavMeshUpdated
    {
        get => _NavMeshUpdated;
        set
        {
            if (_NavMeshUpdated == value)
            {
                return;
            }
            _NavMeshUpdated = value;
            NavMeshUpdatedOnChanged?.Invoke(this);
        }
    }
    public event PropertyChangeEvent NavMeshUpdatedOnChanged;
    #endregion



    protected Vector2 _StartPosition = Vector2.Inf;
    #region property change event
    public Vector2 StartPosition
    {
        get => _StartPosition;
        set
        {
            if (_StartPosition == value)
            {
                return;
            }
            _StartPosition = value;
            StartPositionOnChanged?.Invoke(this);
        }
    }
    public event PropertyChangeEvent StartPositionOnChanged;
    #endregion


    protected Vector2 _EndPosition = Vector2.Inf;
    #region property change event
    public Vector2 EndPosition
    {
        get => _EndPosition;
        set
        {
            if (_EndPosition == value)
            {
                return;
            }
            _EndPosition = value;
            EndPositionOnChanged?.Invoke(this);
        }
    }
    public event PropertyChangeEvent EndPositionOnChanged;
    #endregion


    protected List<NavigationPoint> _Path;
    #region property change event
    public List<NavigationPoint> Path
    {
        get => _Path;
        set
        {
            if (_Path == value)
            {
                return;
            }
            _Path = value;
            PathOnChanged?.Invoke(this);
        }
    }
    public event PropertyChangeEvent PathOnChanged;
    #endregion




    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        toggleCollisionDisplay.Pressed += () =>
        {
            this.World.DisplayCollider = !this.World.DisplayCollider;
        };

        toggleCollisionlayerButton.ItemSelected += (index) =>
        {
            this.World.DisplayedCollisionLayer = (int)index;
        };

        this.WorldOnChanged += (sender) =>
        {
            while (toggleCollisionlayerButton.ItemCount > 0)
            {
                toggleCollisionlayerButton.RemoveItem(0);
            }
            toggleCollisionlayerButton.AddItem("Base colliders");
            foreach (var layer in World.worldCollisionSteps)
            {
                toggleCollisionlayerButton.AddItem(layer.GetType().Name);
            }

        };


        var baseworld = new WaveFunctionCollapseWorld(64);
        
        for(int y = 0; y<2; y++)
        {
            for (int x = 0; x < 2; x++)
            {
                baseworld.GenerateChunk(x, y);
            }
        }

        var wd = new WorldData(baseworld);
        var subdivide = new SubdivideCollisionWorld(baseworld, 3);
        wd.worldCollisionSteps.Add(subdivide);
        var worldCollision = new ExpandedTileCollisionWorld(subdivide);
        wd.worldCollisionSteps.Add(worldCollision);
        World = wd;

        navMesh = new NavMesh(worldCollision);

        
        TileMapEvents.TileOnMouse += (tmap, tile, offset, buttonEvent) =>
        {
            if (buttonEvent.ButtonIndex == MouseButton.Left && buttonEvent.IsPressed())
            {
                if (buttonEvent.ShiftPressed)
                {
                    this.StartPosition = tile + offset;
                    UpdatePath();
                }
                else if (buttonEvent.AltPressed)
                {
                    this.EndPosition = tile + offset;
                    UpdatePath();
                }
                else
                {
                    var x = tile.X / (worldCollision.ChunkSize()* worldCollision.TileScale());
                    var y = tile.Y / (worldCollision.ChunkSize() * worldCollision.TileScale());
                    navMesh.GenerateChunk((int)x, (int)y);
                    navMesh.GenerateAllRemoteLinks((int)x, (int)y);
                    NavMeshUpdated = !NavMeshUpdated;
                }
            }
        };

        



        /*
        var tc = new TestChunk(data);
        var nc = new NavMeshCreator();
        var l = nc.Create(tc);
        var edges = nc.CreateEdges(l);

        var i = 0;
        foreach(var rect in l)
        {
            tilemap.DrawNavMeshRect(rect, 1, 2, new Vector2I(i % 10, i / 10));

            i += 1;
            i = i % 100;
        }

        graph = new NavMeshGraph();
        graph.LoadChunk(new NavMeshChunk(0, 0, l));


        TileMapEvents.TileOnMouse += (tmap, tile, buttonEvent) =>
        {
            if (buttonEvent.ButtonIndex == MouseButton.Left && buttonEvent.IsPressed())
            {

                //var tile = tilemap.LocalToMap(tilemap.ToLocal(buttonEvent.Position / this.GetViewport().GetCamera2D().Scale));
                GD.Print(tile);
                if (!buttonEvent.AltPressed)
                {
                    start = tile;
                }
                else
                {
                    end = tile;
                }

                var edges = graph.FindPath(start, end);

                tilemap.ClearLayer(4);
                tilemap.ClearLayer(7);

                tilemap.SetCell(7, start, 3, new Vector2I(0, 1));
                tilemap.SetCell(7, end, 3, new Vector2I(0, 2));

                if (edges != null)
                {
                    foreach (var edge in edges)
                    {
                        tilemap.DrawNavMeshRect(edge.to, 4, 3, new Vector2I(3, 0));
                        //tilemap.DrawNavMeshRect(edge.right, 4, 3, new Vector2I(3, 0));
                    }

                    (var leftPoints, var rightPoints) = SimpleStupidFunnel.CreateFunnelPoints(start, end, edges);
                    tilemap.ClearLayer(5);
                    tilemap.ClearLayer(6);

                    foreach (var left in leftPoints)
                    {
                        tilemap.SetCell(5, new Vector2I((int)left.X, (int)left.Y), 3, new Vector2I(8, 0));
                    }

                    foreach (var right in rightPoints)
                    {
                        tilemap.SetCell(6, new Vector2I((int)right.X, (int)right.Y), 3, new Vector2I(9, 0));
                    }
                    ssfStates.Clear();
                    var points = SimpleStupidFunnel.Run(start, end, edges, ref ssfStates);
                    foreach (var point in points)
                    {
                        tilemap.SetCell(7, new Vector2I((int)point.X, (int)point.Y), 3, new Vector2I(6, 0));
                    }
                    ssfindex = 0;
                    UpdateSSFState();

                }
            }
        };
        */

    }


    int mode = 0;


    void UpdatePath()
    {
        GD.Print(StartPosition);
        GD.Print(EndPosition);
        if (StartPosition == Vector2.Inf || EndPosition == Vector2.Inf)
        {
            return;
        }


        var path = navMesh.FindPath(new NavigationPoint(StartPosition.X, StartPosition.Y,0), new NavigationPoint(EndPosition.X, EndPosition.Y, 0));
        if (path != null)
        {
            this.Path = path;
            GD.Print(path.Count);
        }
        else {
            GD.Print("No path");
        }
    }
    

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
    {
    }

    public override void _UnhandledKeyInput(InputEvent @event)
    {
        base._UnhandledKeyInput(@event);
        //SmoothTerrain(5, 5, 2, 1);
        //UpdateMap();

        if (@event is InputEventKey keyEvent)
        {
            if (ssfStates.Count > 0)
            {
                if (keyEvent.IsPressed() && keyEvent.Keycode == Key.Q)
                {
                    ssfindex--;
                    if (ssfindex == 0)
                    {
                        ssfindex = ssfStates.Count - 1;
                    }
                }
                if (keyEvent.IsPressed() && keyEvent.Keycode == Key.E)
                {
                    ssfindex++;
                    ssfindex = ssfindex % ssfStates.Count;
                }
                UpdateSSFState();
            }
        }
    }

    void SetCell(int x, int y, int value)
    {
        var v = value - 1;
        var vy = (int) v / 10;
        var vx = v % 10;
        tilemap.SetCell(0, new Vector2I(x, y), 2, new Vector2I(vx, vy));
    }

    void UpdateMap()
    {


        for (int y = 0; y < HeightData.Length; y++)
        {
            for (int x = 0; x < HeightData[y].Length; x++)
            {
                SetCell(x, y, (int)HeightData[y][x]);
            }
        }
    }

    public void SmoothTerrain1(float cordX, float cordY, int BrushSize, float Opacity)
    {
        var markOffset = new int[][]
        {
            new int[] { 1, 0},
            new int[] { 1, 1},
            new int[] { 0, 1},
            new int[] {-1, 1},
            new int[] {-1, 0},
            new int[] {-1,-1},
            new int[] { 0,-1},
            new int[] { 1,-1},
        };
        
        // TODO iterate over all markOffsets, and combine value etc
    }

    public void SmoothTerrain(float cordX, float cordY, int BrushSize, float Opacity)
    {
        float[,] newHeightData;

        // Note: MapWidth and MapHeight should be equal and power-of-two values 
        newHeightData = new float[(BrushSize * 2) + 1, (BrushSize * 2) + 1];

        for (int x = (int)cordX - BrushSize; x <= cordX + BrushSize; x++)
        {
            for (int y = (int)cordY - BrushSize; y <= cordY + BrushSize; y++)
            {
                int adjacentSections = 0;
                float sectionsTotal = 0.0f;

                if ((x - 1) > 0) // Check to left
                {
                    sectionsTotal += HeightData[y][x - 1];
                    adjacentSections++;

                    if ((y - 1) > 0) // Check up and to the left
                    {
                        sectionsTotal += HeightData[y-1][x - 1];
                        adjacentSections++;
                    }

                    if ((y + 1) < cordY + BrushSize) // Check down and to the left
                    {
                        sectionsTotal += HeightData[y+1][x - 1];
                        adjacentSections++;
                    }
                }

                if ((x + 1) < cordX + BrushSize) // Check to right
                {
                    sectionsTotal += HeightData[y][x + 1];
                    adjacentSections++;

                    if ((y - 1) > 0) // Check up and to the right
                    {
                        sectionsTotal += HeightData[y-1][x + 1];
                        adjacentSections++;
                    }

                    if ((y + 1) < cordY + BrushSize) // Check down and to the right
                    {
                        sectionsTotal += HeightData[y+1][x + 1];
                        adjacentSections++;
                    }
                }

                if ((y - 1) > 0) // Check above
                {
                    sectionsTotal += HeightData[y-1][x];
                    adjacentSections++;
                }

                if ((y + 1) < cordY + BrushSize) // Check below
                {
                    sectionsTotal += HeightData[y+1][x];
                    adjacentSections++;
                }

                newHeightData[y, x] = (HeightData[y][x] + (sectionsTotal / adjacentSections)) * Opacity;//0.5f;
            }
        }


        for (int x = (int)cordX - BrushSize; x <= cordX + BrushSize; x++)
        {
            for (int y = (int)cordY - BrushSize; y <= cordY + BrushSize; y++)

                HeightData[(int)cordY - BrushSize + y][(int)cordX - BrushSize + x] = newHeightData[y, x];

        }

    }

    [Export]
    public TileMap tilemap;

    [Export]
    public TileMapEvents TileMapEvents;

    public Vector2I start;
    public Vector2I end;

    List<SSFState> ssfStates = new List<SSFState>();
    int ssfindex = 0;

    float zoom;
    public float Zoom
    {
        get { return zoom; }
        set { zoom = value;
        }
    }

    [Export]
    public Line2D pathLine;

    [Export]
    public Line2D funnelLine;

    [Export]
    public Line2D leftLine;

    [Export]
    public Line2D rightLine;

    [Export]
    public Label helpLabel;

    [Export]
    public PointDrawer drawer;

    [Export]
    public Button toggleCollisionDisplay;

    [Export]
    public OptionButton toggleCollisionlayerButton;

    public void UpdateSSFState()
    {
        var state = ssfStates[ssfindex];
        helpLabel.Text = $"Left index: {state.leftIndex}\n"+
            $"Right Index: {state.rightIndex}\n"+
            $"Help: {state.help}";

        pathLine.ClearPoints();
        funnelLine.ClearPoints();
        leftLine.ClearPoints();
        rightLine.ClearPoints();
        drawer.ClearPoints();

        if (state.pointList != null)
        {
            foreach (var p in state.pointList) {
                pathLine.AddPoint(new Vector2(p.X*32, p.Y*32));
            }
        }

        drawer.AddPoint(state.funnel, Colors.Orange);

        if(state.funnelSide != Vector2.Zero)
        {
            funnelLine.AddPoint(state.funnel * 32);
            funnelLine.AddPoint((state.funnel+ state.funnelSide)*32);
        }


        leftLine.AddPoint(state.funnel*32);
        leftLine.AddPoint((state.funnel + state.left) * 32);

        rightLine.AddPoint(state.funnel * 32);
        rightLine.AddPoint((state.funnel + state.right) * 32);


    }

}
