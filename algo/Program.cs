using System;
using System.IO;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace RouteOptimizer
{
    public struct Point
    {
        public double X { get; set; }
        public double Y { get; set; }
    }

    public class Node : IComparable<Node>
    {
        public int To { get; set; }
        public double Distance { get; set; }
        public double Speed { get; set; }

        public int CompareTo(Node other)
        {
            return To.CompareTo(other.To);
        }
    }

    public struct Query
    {
        public Point P1 { get; set; }
        public Point P2 { get; set; }
        public double R { get; set; }
    }

    public class QueryResult
    {
        public List<int> Path { get; set; } = new List<int>();
        public double TotalTimeMins { get; set; } = 0;
        public double TotalDistanceKm { get; set; } = 0;
        public double WalkingDistanceKm { get; set; } = 0;
        public double VehicleDistanceKm { get; set; } = 0;
    }

    // Custom PriorityQueue implementation
    public class CustomPriorityQueue<T> where T : IComparable<T>
    {
        private List<T> _heap;
        private readonly bool _isMinHeap;

        public CustomPriorityQueue(bool isMinHeap = true)
        {
            _heap = new List<T>();
            _isMinHeap = isMinHeap;
        }

        public int Count => _heap.Count;

        public bool IsEmpty => _heap.Count == 0;

        public void Reserve(int capacity)
        {
            if (_heap.Capacity < capacity)
            {
                _heap.Capacity = capacity;
            }
        }

        public void Clear()
        {
            _heap.Clear();
        }

        public void Push(T item)
        {
            _heap.Add(item);
            SiftUp(_heap.Count - 1);
        }

        public T Peek()
        {
            if (IsEmpty)
                throw new InvalidOperationException("Priority queue is empty");
            return _heap[0];
        }

        public T Pop()
        {
            if (IsEmpty)
                throw new InvalidOperationException("Priority queue is empty");

            T result = _heap[0];
            _heap[0] = _heap[_heap.Count - 1];
            _heap.RemoveAt(_heap.Count - 1);

            if (!IsEmpty)
            {
                SiftDown(0);
            }

            return result;
        }

        private void SiftUp(int index)
        {
            while (index > 0)
            {
                int parentIndex = (index - 1) / 2;
                if (Compare(_heap[index], _heap[parentIndex]) >= 0)
                    break;

                Swap(index, parentIndex);
                index = parentIndex;
            }
        }

        private void SiftDown(int index)
        {
            int count = _heap.Count;
            while (true)
            {
                int smallest = index;
                int leftChild = 2 * index + 1;
                int rightChild = 2 * index + 2;

                if (leftChild < count && Compare(_heap[leftChild], _heap[smallest]) < 0)
                    smallest = leftChild;

                if (rightChild < count && Compare(_heap[rightChild], _heap[smallest]) < 0)
                    smallest = rightChild;

                if (smallest == index)
                    break;

                Swap(index, smallest);
                index = smallest;
            }
        }

        private int Compare(T a, T b)
        {
            return _isMinHeap ? a.CompareTo(b) : b.CompareTo(a);
        }

        private void Swap(int i, int j)
        {
            T temp = _heap[i];
            _heap[i] = _heap[j];
            _heap[j] = temp;
        }
    }

    public class OutputWriter
    {
        private readonly string _fileName;
        private readonly List<QueryResult> _results;

        public OutputWriter(string fileName)
        {
            _fileName = fileName;
            _results = new List<QueryResult>();
        }

        public void AddResult(QueryResult result)
        {
            _results.Add(result);
        }

        public void WriteToFile()
        {
            try
            {
                using (StreamWriter writer = new StreamWriter(_fileName))
                {
                    DateTime start = DateTime.Now;

                    foreach (var result in _results)
                    {
                        // Write path with spaces between nodes
                        for (int i = 0; i < result.Path.Count; i++)
                        {
                            writer.Write(result.Path[i]);
                            if (i != result.Path.Count - 1)
                                writer.Write(" ");  // Space between nodes
                        }
                        writer.WriteLine();

                        // Write metrics with 2 decimal precision
                        writer.WriteLine($"{result.TotalTimeMins:F2} mins");
                        writer.WriteLine($"{result.TotalDistanceKm:F2} km");
                        writer.WriteLine($"{result.WalkingDistanceKm:F2} km");
                        writer.WriteLine($"{result.VehicleDistanceKm:F2} km");
                        writer.WriteLine();  // Extra blank line between results
                    }

                    DateTime mid = DateTime.Now;
                    TimeSpan midDuration = mid - start;
                    writer.WriteLine($"{midDuration.TotalMilliseconds:F0} ms");
                    writer.WriteLine();

                    DateTime end = DateTime.Now;
                    TimeSpan totalDuration = end - start;
                    writer.WriteLine($"{totalDuration.TotalMilliseconds:F0} ms");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error: Unable to open output file. {ex.Message}");
            }
        }

        public void PrintToTerminal()
        {
            foreach (var result in _results)
            {
                // Print path with spaces between nodes
                for (int i = 0; i < result.Path.Count; i++)
                {
                    Console.Write(result.Path[i]);
                    if (i != result.Path.Count - 1)
                        Console.Write(" ");  // Space between nodes
                }
                Console.WriteLine();

                // Write metrics with 2 decimal precision
                Console.WriteLine($"{result.TotalTimeMins:F2} mins");
                Console.WriteLine($"{result.TotalDistanceKm:F2} km");
                Console.WriteLine($"{result.WalkingDistanceKm:F2} km");
                Console.WriteLine($"{result.VehicleDistanceKm:F2} km");
                Console.WriteLine();  // Extra blank line between results
            }
        }
    }

    public class RouteCalculator
    {
        private static double GetMinutes(double distance, double speed)
        {
            return (distance / speed) * 60.0;
        }

        private static double Euclidean(Point p1, Point p2)
        {
            double dx = p1.X - p2.X;
            double dy = p1.Y - p2.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        private static List<int> FindClosestIntersections(Point p, List<Point> pts, double r)
        {
            int n = pts.Count;
            List<int> result = new List<int>();

            for (int i = 0; i < n; i++)
            {
                double dist = Euclidean(p, pts[i]);
                if (dist <= r)
                    result.Add(i);
            }
            return result;
        }

        // For Dijkstra algorithm
        private class DijkstraNode : IComparable<DijkstraNode>
        {
            public double Cost { get; set; }
            public int NodeId { get; set; }

            public int CompareTo(DijkstraNode other)
            {
                return Cost.CompareTo(other.Cost);
            }
        }

        private static (List<double> distances, List<int> parents) MultiSourceDijkstra(
            List<int> sources, Point start, List<Point> pts, List<List<Node>> graph)
        {
            int n = graph.Count;
            List<double> distances = Enumerable.Repeat(double.MaxValue, n + 1).ToList();
            List<int> parents = Enumerable.Repeat(-1, n + 1).ToList();

            // Using our custom priority queue
            CustomPriorityQueue<DijkstraNode> pq = new CustomPriorityQueue<DijkstraNode>();
            pq.Reserve(n); // Reserve space for efficiency

            foreach (var src in sources)
            {
                distances[src] = GetMinutes(Euclidean(pts[src], start), 5);
                pq.Push(new DijkstraNode { Cost = distances[src], NodeId = src });
            }

            while (!pq.IsEmpty)
            {
                var current = pq.Pop();
                double currentCost = current.Cost;
                int currentNode = current.NodeId;

                if (currentCost > distances[currentNode])
                    continue;

                foreach (var edge in graph[currentNode])
                {
                    int childNode = edge.To;
                    double newDistance = GetMinutes(edge.Distance, edge.Speed);
                    newDistance += currentCost;

                    if (newDistance < distances[childNode])
                    {
                        distances[childNode] = newDistance;
                        parents[childNode] = currentNode;
                        pq.Push(new DijkstraNode { Cost = newDistance, NodeId = childNode });
                    }
                }
            }
            return (distances, parents);
        }

        public static QueryResult ProcessQuery(Query query, List<Point> pts, List<List<Node>> graph)
        {
            QueryResult result = new QueryResult();

            List<int> startingPoints = FindClosestIntersections(query.P1, pts, query.R);
            List<int> endingPoints = FindClosestIntersections(query.P2, pts, query.R);

            // Early return if no valid paths
            if (startingPoints.Count == 0 || endingPoints.Count == 0)
            {
                return result; // Return empty result
            }

            var (distances, parents) = MultiSourceDijkstra(startingPoints, query.P1, pts, graph);

            int destination = -1;
            double minDistance = double.MaxValue;

            foreach (var endPoint in endingPoints)
            {
                double dist = GetMinutes(Euclidean(pts[endPoint], query.P2), 5);
                if (dist + distances[endPoint] < minDistance)
                {
                    destination = endPoint;
                    minDistance = dist + distances[endPoint];
                }
            }

            // If no path found
            if (destination == -1 || minDistance == double.MaxValue)
            {
                return result;
            }

            // Reconstruct path
            for (int node = destination; node != -1; node = parents[node])
                result.Path.Add(node);
            result.Path.Reverse();

            // Calculate walking distance
            result.WalkingDistanceKm = Euclidean(pts[result.Path[0]], query.P1) + Euclidean(pts[result.Path[result.Path.Count - 1]], query.P2);
            result.TotalTimeMins += GetMinutes(result.WalkingDistanceKm, 5);

            // Calculate vehicle distance
            for (int i = 0; i + 1 < result.Path.Count; i++)
            {
                int currentNode = result.Path[i];
                int nextNode = result.Path[i + 1];

                // Find the edge between currentNode and nextNode
                Node? edge = null;
                foreach (var e in graph[currentNode])
                {
                    if (e.To == nextNode)
                    {
                        edge = e;
                        break;
                    }
                }

                if (edge != null)
                {
                    result.TotalTimeMins += GetMinutes(edge.Distance, edge.Speed);
                    result.VehicleDistanceKm += edge.Distance;
                }
            }

            result.TotalDistanceKm = result.WalkingDistanceKm + result.VehicleDistanceKm;
            return result;
        }
    }

    public class Program
    {
        public static void Main(string[] args)
        {
            Console.Write("Enter map data file name: ");
            string mapFileName = Console.ReadLine();

            Console.Write("Enter queries file name: ");
            string queryFileName = Console.ReadLine();

            Console.Write("Enter output file name: ");
            string outputFileName = Console.ReadLine();

            Stopwatch progTimer = new Stopwatch();
            progTimer.Start();

            try
            {
                string[] mapLines = File.ReadAllLines(mapFileName + ".txt");
                string[] queryLines = File.ReadAllLines(queryFileName + ".txt");

                int n = int.Parse(mapLines[0]);
                List<Point> intersections = new List<Point>(n);

                // Initialize intersections list with default values
                for (int i = 0; i < n; i++)
                {
                    intersections.Add(new Point());
                }

                // Parse intersection data
                for (int i = 1; i <= n; i++)
                {
                    string[] parts = mapLines[i].Split();
                    int id = int.Parse(parts[0]);
                    double x = double.Parse(parts[1]);
                    double y = double.Parse(parts[2]);
                    intersections[id] = new Point { X = x, Y = y };
                }

                // Parse road data
                int m = int.Parse(mapLines[n + 1]);
                List<List<Node>> graph = new List<List<Node>>(n);

                // Initialize graph
                for (int i = 0; i < n; i++)
                {
                    graph.Add(new List<Node>());
                }

                // Parse road connections
                for (int i = 0; i < m; i++)
                {
                    string[] parts = mapLines[n + 2 + i].Split();
                    int from = int.Parse(parts[0]);
                    int to = int.Parse(parts[1]);
                    double dist = double.Parse(parts[2]);
                    double speed = double.Parse(parts[3]);

                    graph[from].Add(new Node { To = to, Distance = dist, Speed = speed });
                    graph[to].Add(new Node { To = from, Distance = dist, Speed = speed });
                }

                // Sort adjacency lists
                foreach (var edges in graph)
                {
                    edges.Sort();
                }

                // Parse queries
                int q = int.Parse(queryLines[0]);
                List<Query> queries = new List<Query>(q);

                for (int i = 0; i < q; i++)
                {
                    string[] parts = queryLines[i + 1].Split();
                    double x1 = double.Parse(parts[0]);
                    double y1 = double.Parse(parts[1]);
                    double x2 = double.Parse(parts[2]);
                    double y2 = double.Parse(parts[3]);
                    double r = double.Parse(parts[4]) / 1000.0;

                    queries.Add(new Query
                    {
                        P1 = new Point { X = x1, Y = y1 },
                        P2 = new Point { X = x2, Y = y2 },
                        R = r
                    });
                }

                OutputWriter writer = new OutputWriter(outputFileName + ".txt");

                foreach (var query in queries)
                {
                    QueryResult result = RouteCalculator.ProcessQuery(query, intersections, graph);
                    writer.AddResult(result);
                }

                writer.WriteToFile();

                progTimer.Stop();
                Console.WriteLine($"Total time: {progTimer.ElapsedMilliseconds} ms");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error: {ex.Message}");
            }
        }
    }
}