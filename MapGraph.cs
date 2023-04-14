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

    //� ������ ������ �� ����� ������� (���� ������) ������ ������ ������, ��� ������� ����� ����, ����� ������� �� �������,
    //� ������ ������ ������ ���� ������� ������
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

    //(������������� ���������� �� ��������� ������� ���������� ���������� �� ������ �� ������ �� ���������
    private static double[] shortestPathLengths = new double[nodeCount];
    private static bool[] visitedNodes = new bool[nodeCount];  //������������� ���������� �� ��������� ������� ���������� ������
    //������ ��� ������ ��������� ����� ��� ������� ����, �� �������� �� ������ � ������� (��� �������������� ���������� �����)
    private static int[] previoslyMinNodes = new int[nodeCount];

    private static int getMinDistanceNodeIndex()
    {
        double minDistance = double.PositiveInfinity;
        int minIndex = -1; // ���� �� ������� ������� -1, ������ ������������ ����� �� ��������

        for (int currNode = 0; currNode < nodeCount; currNode++) //��� ������ �������
        {
            if (visitedNodes[currNode] == false) // ���� ������� �� ��������
            {
                // ���� ���������� �� ������� ������� ������ ��� ���������� ������������ ����������
                if (shortestPathLengths[currNode].CompareTo(minDistance) < 0)
                {
                    minDistance = shortestPathLengths[currNode]; //���������� ����������� ����������
                    minIndex = currNode; //���������� � �������� ������� ������� � ����������� ����������� �� �� ������ ������� �������
                }
            }
        }
        return minIndex;
    }

    public static void DijkstraPathsFinding(char startingNode)
    {
        int startingNodeIndex = Array.IndexOf(nodeNames, startingNode);
        //������������� ������� ���������� ���������� �� ��������� �������, ������� ���������� ������ � ������� ���������� ����������� ������
        for (int currNode = 0; currNode < nodeCount; currNode++) 
        {
            if (currNode != startingNodeIndex) // ���� ������� ������� �� ���������
                shortestPathLengths[currNode] = double.PositiveInfinity; // � ��������� ���� ���� �����������
            else
                shortestPathLengths[startingNodeIndex] = 0; //���������� �� ��������� ������� == 0
            visitedNodes[currNode] = false; //��� ���� ��� �� ��������
            //������������� ������� ���������� ����������� ������ ������� ��������� �������
            previoslyMinNodes[currNode] = startingNodeIndex;
        }

        int currMinDistanceNodeIndex;
        // ���� �������� ������������ �������, �������� ������� � ����������� ����������� �� ���������
        while ((currMinDistanceNodeIndex = getMinDistanceNodeIndex()) != -1)
        {
            foreach (var Edge in Edges) //��� ������� �����
            {
                bool AdjacentWithFirstNode = nodeNames[currMinDistanceNodeIndex] == Edge.secondNode; //������ � ������ �������� �����
                bool AdjacentWithSecondNode = nodeNames[currMinDistanceNodeIndex] == Edge.firstNode; //������ �� ������ �������� ����� 
                if (AdjacentWithFirstNode || AdjacentWithSecondNode)  //���� ����� ������
                {
                    //���������� �� ��������� ������� �� ������� ������� � �������� � ����������� ����������� �� ���������
                    double distanceToAdjacentNode = shortestPathLengths[currMinDistanceNodeIndex] + Edge.distance;
                    int indexAdjacentNode = 0; //������ ������� �������
                    if (AdjacentWithFirstNode) // ���� ������ � ������ �������� �����
                        indexAdjacentNode = Array.IndexOf(nodeNames, Edge.firstNode);  //�������� ������ ������ ������� �����
                    else if (AdjacentWithSecondNode) //���� ������ �� ������ �������� �����
                        indexAdjacentNode = Array.IndexOf(nodeNames, Edge.secondNode); //�������� ������ ������ ������� �����
                    //���� ���������� �� ������� ������� ������� ����� ������� ����������� ������� ������ ���������� �� ������� �������
                    if (distanceToAdjacentNode.CompareTo(shortestPathLengths[indexAdjacentNode]) < 0) 
                    {
                        //�������� ���������� �� ��������� ������� �� ������� �������
                        shortestPathLengths[indexAdjacentNode] = distanceToAdjacentNode;
                        //���������� � �������� ���������� ����������� ������� ��� ������� ������� ������� ������� � ����������� ����������� �� ���������
                        previoslyMinNodes[indexAdjacentNode] = currMinDistanceNodeIndex; 
                    }
                }
            }
            visitedNodes[currMinDistanceNodeIndex] = true; //�������� ������� ������� � ����������� ����������� �� ��������� ��� ����������
        }
        printDistancesAndPathsFrom(startingNodeIndex); //������� � ������� ���������� � ���� �� ��������� ������� ��� ������ �������
    }

    private static void printDistancesAndPathsFrom(int startNodeIndex)  //�������������� � ����� ���������� ����� � ����������
    {
        for (int currNode = 0; currNode < nodeCount; currNode++)  // ��� ������ �������
        {
            if (currNode != startNodeIndex) // ����� ��������� �������
            {
                string path = nodeNames[currNode].ToString();  // ��� ������� ������� �������� � ������ ����
                //�������� ���������� ������� � ����������� �����������, �� ������� ������ � �������
                int prevNodeIndex = previoslyMinNodes[currNode]; 
                while (prevNodeIndex != startNodeIndex)  //���� ���������� ������� � ����������� ����������� �� ����� ���������
                {
                    path += nodeNames[prevNodeIndex];  //�������� ��� ���������� �������  � ����������� ����������� � ������ ����
                    //����� ���������� ������� � ����������� ����������� ��� ������� ������� �  � ����������� �����������
                    prevNodeIndex = previoslyMinNodes[prevNodeIndex];  
                }
                path += nodeNames[startNodeIndex]; //�������� � ������ ���� ��������� �������
                path = new string(path.ToCharArray().Reverse().ToArray()); // ����������� ������ ����
                Debug.Log("Distance from " + nodeNames[startNodeIndex] + " to " + nodeNames[currNode] + " equals to " +
                            shortestPathLengths[currNode] + " and path equals to " + path);  //������� ���������� � ����
            }
        }
    }
}
