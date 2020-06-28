#ifndef HEAP_H
#define HEAP_H

#include <astar_global_planner/Node.h>

namespace global_planner{
    /**
     * @class Heap
     * @brief A list of objects of type Node which are sorted specifically to quickly find the object with user-defined highest priority
     */
    class Heap{
    public:
        /**
         * @brief Constructor of the class
         * @param maxHeapSize Size of the array of items
         */
        Heap(int maxHeapSize);

        /**
         * @brief Adds an item to the heap and places it to its right position
         * @param item Item to add to the heap
         */
        void Add(Node* item);

        /**
         * @brief Removes first item of heap which has the highest priority
         * @return Item with highest priority
         */
        Node* RemoveFirstItem();

        /**
         * @brief Resorts the heap with the passed item
         * @param item Item to start the sorting with
         */
        void UpdateItem(Node* item);

        /**
         * @brief Checks if the heap has an item or not
         * @param item Item to check
         * @return True if the item exists in the heap, false otherwise
         */
        bool Contains(Node* item);

        /**
         * @brief Moves the passed item downward to sort the heap
         * @param item Item to move through heap
         */
        void SortDown(Node* item);

        /**
         * @brief Moves the passed item upward to sort the heap
         * @param item Item to move through heap
         */
        void SortUp(Node* item);

        /**
         * @brief Swap two items of the heap
         * @param itemA First item
         * @param itemB Second item
         */
        void Swap(Node* itemA, Node* itemB);

        Node** items; /**<Array of sorted items */
        int Count; /**<Current number of items stored in the array */
    };
}

#endif