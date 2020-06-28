#include <astar_global_planner/Heap.h>

namespace global_planner{

    Heap::Heap(int maxHeapSize){
        items = new Node*[maxHeapSize];
    }

    void Heap::Add(Node* item){
        item->HeapIndex = Count;
        items[Count] = item;
        SortUp(item);
        Count++;
    }

    Node* Heap::RemoveFirstItem(){
        Node* firstItem = items[0];
        items[0] = items[--Count];
        items[0]->HeapIndex = 0;
        SortDown(items[0]);
        return firstItem;
    }

    void Heap::UpdateItem(Node* item){
        SortUp(item);
    }

    bool Heap::Contains(Node* item){
        return (items[item->HeapIndex] == item);
    }

    void Heap::SortDown(Node* item){
        while (true)
        {
            int childIndexLeft = 2 * item->HeapIndex + 1;
            int childIndexRight = 2 * item->HeapIndex + 2;
            int swapIndex = 0;

            if (childIndexLeft < Count)
            {
                swapIndex = childIndexLeft;
                if (childIndexRight < Count)
                {
                    if (items[childIndexLeft]->CompareTo(items[childIndexRight]) < 0)
                    {
                        swapIndex = childIndexRight;
                    }
                }
                if (item->CompareTo(items[swapIndex]) < 0)
                {
                    Swap(item, items[swapIndex]);
                }
                else return;
            }
            else
            {
                return;
            }
            
        }
    }

    void Heap::SortUp(Node* item){
        int parentIndex = (item->HeapIndex - 1) / 2;

        while (true)
        {
            Node* parentItem = items[parentIndex];
            if (item->CompareTo(parentItem) > 0)
            {
                Swap(item, parentItem);
            }
            else break;
            parentIndex = (item->HeapIndex - 1) / 2;
        }
    }

    void Heap::Swap(Node* itemA, Node* itemB){
        items[itemA->HeapIndex] = itemB;
        items[itemB->HeapIndex] = itemA;
        int itemAIndex = itemA->HeapIndex;
		itemA->HeapIndex = itemB->HeapIndex;
		itemB->HeapIndex = itemAIndex;
    }

}