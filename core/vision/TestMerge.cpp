#include "TestMerge.h"

TestMerge::TestMerge() {
  // Constructor
  std::cout << "Constructor" << std::endl; 
  createFakeVPAData(vpav_);
  mergeRuns(blobs_);
  
  
}

TestMerge::~TestMerge() {
  // Default Destructor
}

void TestMerge::createFakeVPAData(std::vector<VisionPointAlt>& vpav) {
    std::cout << "create" << std::endl; 
    std::vector<unsigned char> test_data;
    getTestFile(test_data);
    int rows = 100;
    int cols = 100;
    int curr_row = 0;
    int curr_col = 0;
  unsigned char curr_color = 3;
  int run_length = 0;
  VisionPointAlt* vpa = new VisionPointAlt(0,0,(unsigned char)0);
  vpa->color = 0;
  //std::cout << "test color: " << (int)vpa->color << std::endl;
    for (int i = 0; i < test_data.size(); i++) {
    //std::cout << "Data at " << i<< ": " << (int)test_data.at(i) << std::endl;
    if (std::floor(i/cols) == curr_row && test_data.at(i) == vpa->color) {
      //std::cout << "1" << std::endl;
      //extend run
      run_length++;
    } else if (std::floor(i/cols) == (curr_row + 1) || test_data.at(i) != vpa->color) {
      //next row, close run 
      vpa->yf = vpa->yi + run_length;
      vpa->xf = vpa->xi;
      vpa->dy = run_length;
      vpa->dx = 0;
      //std::cout << "Color Pushed: " << (int)vpa->color << std::endl;
      vpav.push_back((*vpa));
      run_length = 0;
      delete vpa;
      vpa = new VisionPointAlt(std::floor(i/cols),i % cols,test_data.at(i));
      vpa->color = test_data.at(i);
      //std::cout << "Color: " << (int)vpa->color << std::endl;
      vpa->xi = std::floor(i/cols);
      vpa->yi = i % cols;
      vpa->isValid = true;
    } 
        curr_row = std::floor(i/cols);
        curr_col = i % cols;
    curr_color = test_data.at(i);

    }
  std::cout << "Number of VPA in array at end of method: " << vpav.size() << std::endl;
}

void TestMerge::getTestFile(std::vector<unsigned char>& test_data) {
  std::cout << "Get file" << std::endl; 
    std::string line;
    ifstream myfile ("testData.txt");
    if (myfile.is_open()) {
        while ( getline (myfile,line) ) {
            std::stringstream linestream(line);
            std::string value;

            while(getline(linestream,value,',')) {
                test_data.push_back((unsigned char)std::atoi(value.c_str()));
            }
        }
    myfile.close();
    std::cout << "Number of test points: " <<test_data.size() << std::endl; 
    }   
    else std::cout << "Unable to open file" << std::endl; 

}


void TestMerge::mergeRuns(std::vector<Blob>& blobs) {
  std::cout << "Merge " << vpav_.size() << " runs" << std::endl; 
  int counter = 0;
  // No default constructor for VisionPointAlt, so you have to have a dummy for the struct
  VisionPointAlt dummy(0,0,0);  
  // Just a dummy as the initial. Might need to review this
  MergeNode node = {dummy,NULL,NULL};
  int vpa_num = vpav_.size();
  int cRow = 0;
  int pvRow;
  std::vector<MergeNode> adjRowCurr;
  std::vector<MergeNode> adjRowPrev;
    for(std::vector<VisionPointAlt>::const_iterator iter = vpav_.begin(); iter !=vpav_.end(); iter++) {
    //std::cout << "VPA #" << counter << ":" << std::endl;
    //std::cout << "Run length: " << (*iter).dy << "    Run color: " << (int)(*iter).color << std::endl;
    node.data = (*iter);
    node.parent = &node;
    node.rank = vpa_num-counter;
    counter++;
    if ((*iter).xi != cRow) {
      // Moved to next row
      cRow = node.data.xi;
      adjRowPrev.clear();
      adjRowPrev.insert(std::end(adjRowPrev), std::begin(adjRowCurr), std::end(adjRowCurr));
      adjRowCurr.clear();
    }
    adjRowCurr.push_back(node);
    checkAdj(node, adjRowPrev);
    
    }


}

void TestMerge::checkAdj(MergeNode node, std::vector<MergeNode> adjRowPrev) {
  //not filled in yet
    for(std::vector<VisionPointAlt>::const_iterator iter = adjRowPrev.begin(); iter !=adjRowPrev.end(); iter++) {
    if ((*iter).color == node.color)
    {
      if (((node.xi >= (*iter).xi) && node.xi <= (*iter).xf) || ((node.xf >= (*iter).xi) && node.xf <= (*iter).xf) || ((node.xf >= (*iter).xf) && node.xi <= (*iter).xi))
      {
        node.parent = (*iter).parent
      }
    }
  // TODO: Fill in Union part
}

TestMerge::MergeNode * TestMerge::findParent(MergeNode node) {
  MergeNode *nodePtr = &node;
  if (node.parent != nodePtr) {
    node.parent = findParent(*(node.parent)); //Recursive loop to find parent
  }
  return node.parent;
}

void TestMerge::unionByRank(MergeNode& a, MergeNode& b) {
  MergeNode *rootA = findParent(a);
  MergeNode *rootB = findParent(b);
  MergeNode *temp;
  if (rootA == rootB) {
    //Already merged, return
    return;
  }
  if ((*rootA).rank < (*rootB).rank) {
    // Swap parents
    temp = rootA;
    rootA = rootB;
    rootB = temp;
    temp = NULL;
  }

  (*rootB).parent = rootA;  // Put the parent back
  if ((*rootA).rank == (*rootB).rank) {
    (*rootA).rank++;  // Increment rank
  }
}


int main(int argc, char **argv) {
  std::cout << "Main" << std::endl; 
  TestMerge* tm = new TestMerge();
    return 0;
}

