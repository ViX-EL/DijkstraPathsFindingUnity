# DijkstraPathsFindingUnity
Моя реализация алгоритма Дейкстры для поиска пути в Unity (алгоритм ищет кратчайшие пути от выбранного до всех остальных узлов).
Пути и расстояния выводятся в лог.
Данное решение было написано во время решения одной из задач на платформе https://startgame.rsv.ru/.

Пошаговое описание алгоритма:
1. Инициализация массива посещённых узлов.
2. Инициализация массива кратчайших расстояний от начального узла.
3. Установить расстояние до начального узла равным 0, а до остальных - бесконечность
4. Установить в списке посещённых узлов для всех узлов false (не посещено).
5. Найдите непосещённый узел с наименьшим текущим расстоянием в качестве текущего узла C.
6. Для каждого соседа N вашего текущего узла C: добавьте текущее расстояние C к весу ребра, соединяющего C-N. Если оно меньше текущего расстояния N, установите его как новое текущее расстояние N.
7. Отметьте текущий узел C как посещенный.
8. Если есть непосещённые узлы, перейдите к шагу 5.
Источник: https://www.codingame.com/playgrounds/1608/shortest-paths-with-dijkstras-algorithm/dijkstras-algorithm

![image](https://user-images.githubusercontent.com/44582410/232083952-ae93ab8a-30a3-49c7-b9fc-8b44664c06a4.png)
