#include "openlist.h"

OpenList::OpenList() { size = 0; }

OpenList::OpenList(int size_) {
    elements.resize(size_);
    size = 0;
    height = size_;
}

void OpenList::resize(int size_) {
    elements.resize(size_);
    height = size_;
    size = 0;
}

OpenList::~OpenList() {
    for (auto elem : elements) {
       if(!elem.empty()) elem.erase(elem.begin(), elem.end());
    }
}

size_t OpenList::get_size() const {
    return size;
}

bool OpenList::is_empty() const {
    if (size == 0) return true;
    return false;
}

bool OpenList::top_key_less_than(Key cur_key) { //compare the minimum key value and current key value
    bool exists = false;
    Key best_key(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    for (size_t i = 0; i < height; i++) {
        if (!elements[i].empty()) {
            exists = true;
            if (elements[i].front()->key < best_key) {
                best_key = elements[i].front()->key;
                top_coord = i;
            }
        }
    }
    if(!exists) return false;
    return best_key < cur_key;
}

Node* OpenList::get() { //return node wit minimum key value
    Node* best = elements[top_coord].front();
    elements[top_coord].pop_front();
    return best;
}


void OpenList::clear() {
    elements.erase(elements.begin(), elements.end());
}

void OpenList::print_elements() const {
    for (auto elem : elements) {
        if (!elem.empty()) {
            for(auto it = elem.begin(); it != elem.end(); ++it) {
                std::cout << (**it) << "<-" << *(*it)->parent << (*it)->key.k1 << ' ' << (*it)->key.k2 << " ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << "end\n";
}

void OpenList::put (Node* item) { //add node to OPEN list or renew it's key value, is it is already there
    if (elements[item->i].empty()) {
        elements[item->i].push_back(item);
        ++size;
        return;
    }
    std::list<Node*>::iterator pos = elements[item->i].end();
    bool pos_found = false;

    for(auto it = elements[item->i].begin(); it != elements[item->i].end(); ++it) {
        if (item->key < (*it)->key && (!pos_found)) {
            pos = it;
            pos_found = true;
        }
        if (**it == *item) {
            if (!(item->key < (*it)->key)) return;
            else {
                if(pos == it) {
                    *it = item;
                    return;
                }
                elements[item->i].erase(it);
                --size;
                break;
            }
        }
    }
    ++size;
    elements[item->i].insert(pos, item);
}

void OpenList::remove_if(Node* item) {
    elements[item->i].remove_if([item](Node* curr) { return *curr == *item; });
}

bool OpenList::find(Node* item) {
    return std::find_if(elements[item->i].begin(), elements[item->i].end(),[item](Node* curr) { return *curr == *item; }) != elements[item->i].end();
}

void OpenList::remove_all(Node item) {
    elements[item.i].remove_if([item](Node* curr) { return curr->i == item.i && curr->j == item.j; });
}

tinyxml2::XMLElement * OpenList::writeToXml(tinyxml2::XMLElement * element, tinyxml2::XMLNode * child) const {
    Node min;
    for(size_t i = 0; i < height; i++) {
        if(!elements[i].empty() && elements[i].front()->key < min.key) {
            min = *elements[i].front();
        }
    }
    if(min.g != std::numeric_limits<float>::infinity()) {
        element->SetAttribute(CNS_TAG_ATTR_X, min.j);
        element->SetAttribute(CNS_TAG_ATTR_Y, min.i);
        element->SetAttribute(CNS_TAG_ATTR_F, min.rhs);
        element->SetAttribute(CNS_TAG_ATTR_G, min.g);
        element->SetAttribute(CNS_TAG_ATTR_PARX, min.parent->j);
        element->SetAttribute(CNS_TAG_ATTR_PARY, min.parent->i);
        child->InsertEndChild(element);
    }
    for(size_t i = 0; i < height; ++i) {
        if(!elements[i].empty()) {
            for (auto it = elements[i].begin(); it != elements[i].end(); ++it) {
                if (**it != min) {
                    element->SetAttribute(CNS_TAG_ATTR_X, (*it)->j);
                    element->SetAttribute(CNS_TAG_ATTR_Y, (*it)->i);
                    element->SetAttribute(CNS_TAG_ATTR_F, (*it)->rhs);
                    element->SetAttribute(CNS_TAG_ATTR_G, (*it)->g);
                    if ((*it)->g > 0){
                        element->SetAttribute(CNS_TAG_ATTR_PARX, (*it)->parent->j);
                        element->SetAttribute(CNS_TAG_ATTR_PARY, (*it)->parent->i);
                    }
                    child->InsertEndChild(element);
                }
            }
        }
    }
    return element;
}

