from libcpp cimport bool
from libcpp.string cimport string
from libcpp.unordered_set cimport unordered_set
from libcpp.pair cimport pair

cdef extern from * nogil:
    cdef T dynamic_cast[T](void *) except +   # nullptr may also indicate failure
    cdef T static_cast[T](void *)
    cdef T reinterpret_cast[T](void *)
    cdef T const_cast[T](void *)

cdef extern from "<iostream>" namespace "std":
    cdef cppclass istream:
        istream() except +
    cdef cppclass ostream:
        ostream() except +

cdef extern from "<sstream>" namespace "std":
    cdef cppclass istringstream:
        istringstream() except +
        istringstream(string& s) except +
        string str()
        void str(string& s)
    cdef cppclass ostringstream:
        ostringstream() except +
        string str()

cdef extern from "octomap/math/Vector3.h" namespace "octomath":
    cdef cppclass Vector3:
        Vector3() except +
        Vector3(float, float, float) except +
        Vector3(Vector3& other) except +
        float& x()
        float& y()
        float& z()

cdef extern from "octomap/octomap_types.h" namespace "octomap":
    ctypedef Vector3 point3d

cdef extern from "octomap/Pointcloud.h" namespace "octomap":
    cdef cppclass Pointcloud:
        Pointcloud() except +
        void push_back(float, float, float)
        void push_back(point3d* p)

cdef extern from "octomap/OcTreeNode.h" namespace "octomap":
    cdef cppclass OcTreeNode:
        OcTreeNode() except +
        bool createChild(unsigned int i)
        void addValue(float& p)
        bool childExists(unsigned int i)
        float getValue()
        void setValue(float v)
        double getOccupancy()
        void expandNode()
        OcTreeNode* getChild(unsigned int i)
        float getLogOdds()
        void setLogOdds(float l)
        bool hasChildren()
        bool collapsible()
        void deleteChild(unsigned int i)
        bool pruneNode()

cdef extern from "octomap/OcTreeKey.h" namespace "octomap":
    cdef cppclass OcTreeKey:
        cppclass KeyHash:
            pass
        OcTreeKey() except +
        OcTreeKey(unsigned short int a, unsigned short int b, unsigned short int c) except +
        OcTreeKey(OcTreeKey& other)
        unsigned short int& operator[](unsigned int i)

cdef extern from "octomap/OcTreeKey.h" namespace "octomap":
    cdef cppclass KeySet:
        cppclass iterator:
            OcTreeKey& operator*()
            iterator operator++()
            iterator operator--()
            bint operator==(iterator)
            bint operator!=(iterator)
        cppclass reverse_iterator:
            OcTreeKey& operator*()
            iterator operator++()
            iterator operator--()
            bint operator==(reverse_iterator)
            bint operator!=(reverse_iterator)
        cppclass const_iterator(iterator):
            pass
        cppclass const_reverse_iterator(reverse_iterator):
            pass
        KeySet() except +
        KeySet(KeySet&) except +
        #KeySet(key_compare&)
        #KeySet& operator=(KeySet&)
        bint operator==(KeySet&, KeySet&)
        bint operator!=(KeySet&, KeySet&)
        bint operator<(KeySet&, KeySet&)
        bint operator>(KeySet&, KeySet&)
        bint operator<=(KeySet&, KeySet&)
        bint operator>=(KeySet&, KeySet&)
        iterator begin()
        const_iterator const_begin "begin"()
        void clear()
        size_t count(OcTreeKey&)
        bint empty()
        iterator end()
        const_iterator const_end "end"()
        pair[iterator, iterator] equal_range(OcTreeKey&)
        #pair[const_iterator, const_iterator] equal_range(OcTreeKey&)
        void erase(iterator)
        void erase(iterator, iterator)
        size_t erase(OcTreeKey&)
        iterator find(OcTreeKey&)
        const_iterator const_find "find"(OcTreeKey&)
        pair[iterator, bint] insert(OcTreeKey&)
        iterator insert(iterator, OcTreeKey&)
        #void insert(input_iterator, input_iterator)
        #key_compare key_comp()
        iterator lower_bound(OcTreeKey&)
        const_iterator const_lower_bound "lower_bound"(OcTreeKey&)
        size_t max_size()
        reverse_iterator rbegin()
        const_reverse_iterator const_rbegin "rbegin"()
        reverse_iterator rend()
        const_reverse_iterator const_rend "rend"()
        size_t size()
        void swap(KeySet&)
        iterator upper_bound(OcTreeKey&)
        const_iterator const_upper_bound "upper_bound"(OcTreeKey&)

cdef extern from "include_and_setting.h" namespace "octomap":
    cdef cppclass OccupancyOcTreeBase[T]:
        cppclass iterator_base:
            point3d getCoordinate()
            unsigned int getDepth()
            OcTreeKey getIndexKey()
            OcTreeKey& getKey()
            double getSize() except +
            double getX() except +
            double getY() except +
            double getZ() except +
            T& operator*()
            bool operator==(iterator_base &other)
            bool operator!=(iterator_base &other)

        cppclass tree_iterator(iterator_base):
            tree_iterator() except +
            tree_iterator(tree_iterator&) except +
            tree_iterator& operator++()
            bool operator==(tree_iterator &other)
            bool operator!=(tree_iterator &other)
            bool isLeaf() except +

        cppclass leaf_iterator(iterator_base):
            leaf_iterator() except +
            leaf_iterator(leaf_iterator&) except +
            leaf_iterator& operator++()
            bool operator==(leaf_iterator &other)
            bool operator!=(leaf_iterator &other)

        cppclass leaf_bbx_iterator(iterator_base):
            leaf_bbx_iterator() except +
            leaf_bbx_iterator(leaf_bbx_iterator&) except +
            leaf_bbx_iterator& operator++()
            bool operator==(leaf_bbx_iterator &other)
            bool operator!=(leaf_bbx_iterator &other)

cdef extern from "include_and_setting.h" namespace "octomap":
    cdef cppclass OcTree:
        OcTree(double resolution) except +
        OcTree(string _filename) except +
        OcTreeKey adjustKeyAtDepth(OcTreeKey& key, unsigned int depth)
        unsigned short int adjustKeyAtDepth(unsigned short int key, unsigned int depth)
        bool bbxSet()
        size_t calcNumNodes()
        void clear()
        OcTreeKey coordToKey(point3d& coord)
        OcTreeKey coordToKey(point3d& coord, unsigned int depth)
        bool coordToKeyChecked(point3d& coord, OcTreeKey& key)
        bool coordToKeyChecked(point3d& coord, unsigned int depth, OcTreeKey& key)
        bool deleteNode(point3d& value, unsigned int depth)
        bool castRay(point3d& origin, point3d& direction, point3d& end,
                     bool ignoreUnknownCells, double maxRange)
        OcTree* read(string& filename)
        OcTree* read(istream& s)
        bool write(string& filename)
        bool write(ostream& s)
        bool readBinary(string& filename)
        bool readBinary(istream& s)
        bool writeBinary(string& filename)
        bool writeBinary(ostream& s)
        bool isNodeOccupied(OcTreeNode& occupancyNode)
        bool isNodeAtThreshold(OcTreeNode& occupancyNode)
        void insertPointCloud(Pointcloud& scan, point3d& sensor_origin,
                              double maxrange, bool lazy_eval)
        OccupancyOcTreeBase[OcTreeNode].tree_iterator begin_tree(unsigned char maxDepth) except +
        OccupancyOcTreeBase[OcTreeNode].tree_iterator end_tree() except +
        OccupancyOcTreeBase[OcTreeNode].leaf_iterator begin_leafs(unsigned char maxDepth) except +
        OccupancyOcTreeBase[OcTreeNode].leaf_iterator end_leafs() except +
        OccupancyOcTreeBase[OcTreeNode].leaf_bbx_iterator begin_leafs_bbx(point3d &min, point3d &max, unsigned char maxDepth) except +
        OccupancyOcTreeBase[OcTreeNode].leaf_bbx_iterator end_leafs_bbx() except +
        point3d getBBXBounds()
        point3d getBBXCenter()
        point3d getBBXMax()
        point3d getBBXMin()
        OcTreeNode* getRoot()
        size_t getNumLeafNodes()
        double getResolution()
        unsigned int getTreeDepth()
        string getTreeType()
        bool inBBX(point3d& p)
        point3d keyToCoord(OcTreeKey& key)
        point3d keyToCoord(OcTreeKey& key, unsigned int depth)
        unsigned long long memoryFullGrid()
        size_t memoryUsage()
        size_t memoryUsageNode()
        void resetChangeDetection()
        OcTreeNode* search(double x, double y, double z, unsigned int depth)
        OcTreeNode* search(point3d& value, unsigned int depth)
        OcTreeNode* search(OcTreeKey& key, unsigned int depth)
        void setBBXMax(point3d& max)
        void setBBXMin(point3d& min)
        void setResolution(double r)
        size_t size()
        void toMaxLikelihood()
        OcTreeNode* updateNode(double x, double y, double z, float log_odds_update, bool lazy_eval)
        OcTreeNode* updateNode(double x, double y, double z, bool occupied, bool lazy_eval)
        OcTreeNode* updateNode(OcTreeKey& key, float log_odds_update, bool lazy_eval)
        OcTreeNode* updateNode(OcTreeKey& key, bool occupied, bool lazy_eval)
        void updateInnerOccupancy()
        void useBBXLimit(bool enable)
        double volume()

        double getClampingThresMax()
        float getClampingThresMaxLog()
        double getClampingThresMin()
        float getClampingThresMinLog()

        double getOccupancyThres()
        float getOccupancyThresLog()
        double getProbHit()
        float getProbHitLog()
        double getProbMiss()
        float getProbMissLog()

        void setClampingThresMax(double thresProb)
        void setClampingThresMin(double thresProb)
        void setOccupancyThres(double prob)
        void setProbHit(double prob)
        void setProbMiss(double prob)

        void getMetricSize(double& x, double& y, double& z)
        void getMetricMin(double& x, double& y, double& z)
        void getMetricMax(double& x, double& y, double& z)

ctypedef unsigned char uint8_t

cdef extern from "octomap/ColorOcTree.h" namespace "octomap":
    cdef cppclass ColorOcTreeNode:
        cppclass Color:
            Color()
            Color(uint8_t, uint8_t, uint8_t)
            uint8_t b, g, r
        ColorOcTreeNode() except +
        ColorOcTreeNode.Color getAverageChildColor()
        ColorOcTreeNode.Color getColor()
        bool isColorSet()
        void setColor(uint8_t, uint8_t, uint8_t)
        bool createChild(unsigned int i)
        void addValue(float& p)
        bool childExists(unsigned int i)
        float getValue()
        void setValue(float v)
        double getOccupancy()
        void expandNode()
        ColorOcTreeNode* getChild(unsigned int i)
        float getLogOdds()
        void setLogOdds(float l)
        bool hasChildren()
        bool collapsible()
        void deleteChild(unsigned int i)
        bool pruneNode()

cdef extern from "octomap/ColorOcTree.h" namespace "octomap":
    cdef cppclass ColorOcTree:
        ColorOcTree(double resolution) except +
        ColorOcTreeNode* averageNodeColor(OcTreeKey& key, uint8_t, uint8_t, uint8_t)
        ColorOcTreeNode* averageNodeColor(float x, float y, float z, uint8_t, uint8_t, uint8_t)
        OcTreeKey adjustKeyAtDepth(OcTreeKey& key, unsigned int depth)
        unsigned short int adjustKeyAtDepth(unsigned short int key, unsigned int depth)
        bool bbxSet()
        size_t calcNumNodes()
        void clear()
        void computeUpdate(Pointcloud &scan, point3d &origin, KeySet &free_cells, KeySet &occupied_cells, double maxrange)
        OcTreeKey coordToKey(point3d& coord)
        OcTreeKey coordToKey(point3d& coord, unsigned int depth)
        bool coordToKeyChecked(point3d& coord, OcTreeKey& key)
        bool coordToKeyChecked(point3d& coord, unsigned int depth, OcTreeKey& key)
        bool deleteNode(point3d& value, unsigned int depth)
        bool castRay(point3d& origin, point3d& direction, point3d& end,
                     bool ignoreUnknownCells, double maxRange)
        ColorOcTree* read(string& filename)
        ColorOcTree* read(istream& s)
        bool write(string& filename)
        bool write(ostream& s)
        bool readBinary(string& filename)
        bool readBinary(istream& s)
        bool writeBinary(string& filename)
        bool writeBinary(ostream& s)
        bool isNodeOccupied(OcTreeNode& occupancyNode)
        bool isNodeAtThreshold(OcTreeNode& occupancyNode)
        void insertPointCloud(Pointcloud& scan, point3d& sensor_origin,
                              double maxrange, bool lazy_eval)
        ColorOcTreeNode* integrateNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
        ColorOcTreeNode* integrateNodeColor(OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b)
        OccupancyOcTreeBase[ColorOcTreeNode].tree_iterator begin_tree(unsigned char maxDepth) except +
        OccupancyOcTreeBase[ColorOcTreeNode].tree_iterator end_tree() except +
        OccupancyOcTreeBase[ColorOcTreeNode].leaf_iterator begin_leafs(unsigned char maxDepth) except +
        OccupancyOcTreeBase[ColorOcTreeNode].leaf_iterator end_leafs() except +
        OccupancyOcTreeBase[ColorOcTreeNode].leaf_bbx_iterator begin_leafs_bbx(point3d &min, point3d &max, unsigned char maxDepth) except +
        OccupancyOcTreeBase[ColorOcTreeNode].leaf_bbx_iterator end_leafs_bbx() except +
        point3d getBBXBounds()
        point3d getBBXCenter()
        point3d getBBXMax()
        point3d getBBXMin()
        ColorOcTreeNode* getRoot()
        size_t getNumLeafNodes()
        double getResolution()
        unsigned int getTreeDepth()
        string getTreeType()
        bool inBBX(point3d& p)
        point3d keyToCoord(OcTreeKey& key)
        point3d keyToCoord(OcTreeKey& key, unsigned int depth)
        unsigned long long memoryFullGrid()
        size_t memoryUsage()
        size_t memoryUsageNode()
        void resetChangeDetection()
        ColorOcTreeNode* search(double x, double y, double z, unsigned int depth)
        ColorOcTreeNode* search(point3d& value, unsigned int depth)
        ColorOcTreeNode* search(OcTreeKey& key, unsigned int depth)
        void setBBXMax(point3d& max)
        void setBBXMin(point3d& min)
        ColorOcTreeNode* setNodeColor(OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b)
        ColorOcTreeNode* setNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
        void setResolution(double r)
        size_t size()
        void toMaxLikelihood()
        ColorOcTreeNode* updateNode(double x, double y, double z, float log_odds_update, bool lazy_eval)
        ColorOcTreeNode* updateNode(double x, double y, double z, bool occupied, bool lazy_eval)
        ColorOcTreeNode* updateNode(OcTreeKey& key, float log_odds_update, bool lazy_eval)
        ColorOcTreeNode* updateNode(OcTreeKey& key, bool occupied, bool lazy_eval)
        void updateInnerOccupancy()
        void useBBXLimit(bool enable)
        double volume()

        double getClampingThresMax()
        float getClampingThresMaxLog()
        double getClampingThresMin()
        float getClampingThresMinLog()

        double getOccupancyThres()
        float getOccupancyThresLog()
        double getProbHit()
        float getProbHitLog()
        double getProbMiss()
        float getProbMissLog()

        void setClampingThresMax(double thresProb)
        void setClampingThresMin(double thresProb)
        void setOccupancyThres(double prob)
        void setProbHit(double prob)
        void setProbMiss(double prob)

        void getMetricSize(double& x, double& y, double& z)
        void getMetricMin(double& x, double& y, double& z)
        void getMetricMax(double& x, double& y, double& z)
