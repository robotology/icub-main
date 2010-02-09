// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CBAPI_UTIL__H_
#define _CBAPI_UTIL__H_

#include <string>
#include <gtkmm.h>

namespace CB {

  class ResourceList : public Gtk::ScrolledWindow {

  public:
    ResourceList(std::string name);
    virtual ~ResourceList();
    
    class ModelColumns : public Gtk::TreeModel::ColumnRecord {
    public:
      
      ModelColumns() {
	add(m_col_text); 
      }
      
      Gtk::TreeModelColumn<Glib::ustring> m_col_text;
    };
    
    ModelColumns m_Columns;
    
    void addResource(std::string s);
    void clear();
    
    std::string getSelected();
    
    Glib::RefPtr<Gtk::TreeSelection> getTreeSelection() {
      return m_TreeView.get_selection();
    }
    
  protected:
    
    Glib::RefPtr<Gtk::ListStore> m_refListStore;
    Glib::RefPtr<Gtk::TreeSelection> m_sel;
    Gtk::TreeView m_TreeView;
    std::string name;
    
  };
  
  class CBAPITextWindow : public Gtk::ScrolledWindow {
    
  public:

    CBAPITextWindow(std::string fgColor="black", std::string bgColor="white");
    virtual ~CBAPITextWindow();
    
    void append_text(std::string text);
    void prepend_text(std::string text);
    void clear_text();
    void set_adjustment();

  protected:
    
    Gtk::TextView m_TextView;
    std::string fg, bg;
    
  };
  
  class PotentialFunctionInfo {
    
  public:
    
    PotentialFunctionInfo() {}
    ~PotentialFunctionInfo() {}
    
    std::string name;
    std::string space;
    bool hasReference;
  
  };
  
  class JacobianInfo {
    
  public:
    
    JacobianInfo() {}
    ~JacobianInfo() {}
    
    std::string name;
    std::string inputSpace;
    std::string outputSpace;
    
  };
  
  class ResourceInfo {
    
  public:
    
    ResourceInfo() {}
    ~ResourceInfo() {}
    
    std::string name;
    std::string space;
    
  };

}


#endif
