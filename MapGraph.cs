using System;
using System.Linq;
using UnityEngine;

public static class MapGraph
{
    public struct Edge
    {
        public Edge(char firstNode, char secondNode, double distance)
        {
            this.firstNode = firstNode;
            this.secondNode = secondNode;
            this.distance = distance;
        }
        public char firstNode;
        public char secondNode;
        public double distance;
    }

    public static char[] nodeNames { get; } = { Gate.Name[0], Mansion.Name[0], House.Name[0], Quarry.Name[0], 
                                                Archer_arena.Name[0], Castle.Name[0], Stable.Name[0]};

    //В данном случае из имени объекта (поле класса) берётся первый символ, для задания имени узла, можно сделать по другому,
    //в данном случае такими были условия задачи
    public static Edge[] Edges { get; } = {
    new Edge(Gate.Name[0], Mansion.Name[0], Vector2.Distance(Gate.Position, Mansion.Position)),
    new Edge(Mansion.Name[0], House.Name[0], Vector2.Distance(Mansion.Position, House.Position)),
    new Edge(Mansion.Name[0], Archer_arena.Name[0], Vector2.Distance(Mansion.Position, Archer_arena.Position)),
    new Edge(Mansion.Name[0], Quarry.Name[0], Vector2.Distance(Mansion.Position, Quarry.Position)),
    new Edge(House.Name[0], Archer_arena.Name[0], Vector2.Distance(House.Position, Archer_arena.Position)),
    new Edge(House.Name[0], Stable.Name[0], Vector2.Distance(House.Position, Stable.Position)),
    new Edge(Archer_arena.Name[0], Castle.Name[0], Vector2.Distance(Archer_arena.Position, Castle.Position)),
    new Edge(Castle.Name[0], Quarry.Name[0], Vector2.Distance(Castle.Position, Quarry.Position)),
    new Edge(Castle.Name[0], Stable.Name[0], Vector2.Distance(Castle.Position, Stable.Position))};

    private static readonly int nodeCount = nodeNames.Length;

    //(инициализация значениями по умолчанию массива кратчайших расстояний до каждой из вершин от стартовой
    private static double[] shortestPathLengths = new double[nodeCount];
    private static bool[] visitedNodes = new bool[nodeCount];  //инициализация значениями по умолчанию массива посещённых вершин
    //Массив для записи предыдщих узлов для каждого узла, из которого мы попали в текущий (для восстановления кратчайших путей)
    private static int[] previoslyMinNodes = new int[nodeCount];

    private static int getMinDistanceNodeIndex()
    {
        double minDistance = double.PositiveInfinity;
        int minIndex = -1; // если из функции вернётся -1, значит непосещённых узлов не осталось

        for (int currNode = 0; currNode < nodeCount; currNode++) //для каждой вершины
        {
            if (visitedNodes[currNode] == false) // если вершина не посещена
            {
                // если расстояние до текущей вершины меньше уже имеющегося минимального расстояния
                if (shortestPathLengths[currNode].CompareTo(minDistance) < 0)
                {
                    minDistance = shortestPathLengths[currNode]; //установить минимальное расстояние
                    minIndex = currNode; //установить в качестве индекса вершины с минимальным расстоянием до неё индекс текущей вершины
                }
            }
        }
        return minIndex;
    }

    public static void DijkstraPathsFinding(char startingNode)
    {
        int startingNodeIndex = Array.IndexOf(nodeNames, startingNode);
        //инициализация массива кратчайших расстояний от стартовой вершины, массива посещённых вершин и массива предыдущих минимальных вершин
        for (int currNode = 0; currNode < nodeCount; currNode++) 
        {
            if (currNode != startingNodeIndex) // если текущая вершина не стартовая
                shortestPathLengths[currNode] = double.PositiveInfinity; // а остальные узлы пока недостяжимы
            else
                shortestPathLengths[startingNodeIndex] = 0; //расстояние до стартовой вершины == 0
            visitedNodes[currNode] = false; //все узлы ещё не посещены
            //инициализация массива предыдущих минимальных вершин номером стартовой вершины
            previoslyMinNodes[currNode] = startingNodeIndex;
        }

        int currMinDistanceNodeIndex;
        // пока остались непосещённые вершины, получить вершину с минимальным расстоянием от стартовой
        while ((currMinDistanceNodeIndex = getMinDistanceNodeIndex()) != -1)
        {
            foreach (var Edge in Edges) //для каждого ребра
            {
                bool AdjacentWithFirstNode = nodeNames[currMinDistanceNodeIndex] == Edge.secondNode; //смежно с первой вершиной ребра
                bool AdjacentWithSecondNode = nodeNames[currMinDistanceNodeIndex] == Edge.firstNode; //смежно со второй вершиной ребра 
                if (AdjacentWithFirstNode || AdjacentWithSecondNode)  //если ребро смежно
                {
                    //расстояние от стартовой вершины до текущей смежной с вершиной с минимальным расстоянием от стартовой
                    double distanceToAdjacentNode = shortestPathLengths[currMinDistanceNodeIndex] + Edge.distance;
                    int indexAdjacentNode = 0; //индекс смежной вершины
                    if (AdjacentWithFirstNode) // если смежно с первой вершиной ребра
                        indexAdjacentNode = Array.IndexOf(nodeNames, Edge.firstNode);  //получить индекс первой вершины ребра
                    else if (AdjacentWithSecondNode) //если смежно со второй вершиной ребра
                        indexAdjacentNode = Array.IndexOf(nodeNames, Edge.secondNode); //получить индекс второй вершины ребра
                    //если расстояние до текущей смежной вершины через текущёю минимальную вершину меньше имеющегося до текущей смежной
                    if (distanceToAdjacentNode.CompareTo(shortestPathLengths[indexAdjacentNode]) < 0) 
                    {
                        //обновить расстояние от стартовой вершины до текущей смежной
                        shortestPathLengths[indexAdjacentNode] = distanceToAdjacentNode;
                        //установить в качестве предыдущей минимальной вершины для текущей смежной вершины вершину с минимальным расстоянием от стартовой
                        previoslyMinNodes[indexAdjacentNode] = currMinDistanceNodeIndex; 
                    }
                }
            }
            visitedNodes[currMinDistanceNodeIndex] = true; //пометить текущую вершину с минимальным расстоянием до стартовой как посещённую
        }
        printDistancesAndPathsFrom(startingNodeIndex); //вывести в консоль расстояния и пути от стартовой вершины для каждой вершины
    }

    private static void printDistancesAndPathsFrom(int startNodeIndex)  //восстановление и вывод кратчайших путей и расстояний
    {
        for (int currNode = 0; currNode < nodeCount; currNode++)  // для каждой вершины
        {
            if (currNode != startNodeIndex) // кроме стартовой вершины
            {
                string path = nodeNames[currNode].ToString();  // имя текущей вершины добавить в строку пути
                //получить предыдущую вершину с минимальным расстоянием, из которой пришли в текущую
                int prevNodeIndex = previoslyMinNodes[currNode]; 
                while (prevNodeIndex != startNodeIndex)  //пока предыдущая вершина с минимальным расстоянием не равна стартовой
                {
                    path += nodeNames[prevNodeIndex];  //добавить имя предыдущей вершины  с минимальным расстоянием в строку пути
                    //взять предыдущую вершину с минимальным расстоянием для текущей вершины с  с минимальным расстоянием
                    prevNodeIndex = previoslyMinNodes[prevNodeIndex];  
                }
                path += nodeNames[startNodeIndex]; //добавить к строке пути стартовую вершину
                path = new string(path.ToCharArray().Reverse().ToArray()); // перевернуть строку пути
                Debug.Log("Distance from " + nodeNames[startNodeIndex] + " to " + nodeNames[currNode] + " equals to " +
                            shortestPathLengths[currNode] + " and path equals to " + path);  //вывести расстояние и путь
            }
        }
    }
}
