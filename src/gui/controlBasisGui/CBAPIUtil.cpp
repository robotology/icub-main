// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "CBAPIUtil.h"
#include <iostream>

using namespace CB;
using namespace std;

ResourceList::ResourceList(string n) {
  name = n;
  set_policy(Gtk::POLICY_AUTOMATIC,Gtk::POLICY_AUTOMATIC);
  add(m_TreeView);
  m_refListStore = Gtk::ListStore::create(m_Columns);
  m_TreeView.set_model(m_refListStore);
  m_TreeView.append_column(name.c_str(), m_Columns.m_col_text);
  show_all_children();
  Glib::RefPtr<Gtk::TreeSelection> m_sel = m_TreeView.get_selection();
}

ResourceList::~ResourceList() { }

void ResourceList::addResource(string s) {
  Gtk::TreeModel::Row row = *(m_refListStore->append());
  row[m_Columns.m_col_text] = s;
}

string ResourceList::getSelected() {
  m_sel = m_TreeView.get_selection();
  Gtk::TreeModel::iterator iter = m_sel->get_selected();
  if(iter) {
    Gtk::TreeModel::Row row = *iter;
    cout << "Row activated, Name=" << row[m_Columns.m_col_text] << endl;
    Glib::ustring us = row[m_Columns.m_col_text];
    return us.c_str();
  } else {
    return "";
  }
}

void ResourceList::clear() {
  m_refListStore.clear(); 
  m_refListStore = Gtk::ListStore::create(m_Columns);
  m_TreeView.set_model(m_refListStore);
  show_all_children();
}

CBAPITextWindow::CBAPITextWindow(string fgColor, string bgColor) {
  set_policy(Gtk::POLICY_AUTOMATIC,Gtk::POLICY_AUTOMATIC);
  add(m_TextView);
  append_text("");
  m_TextView.set_editable(false);

  fg = fgColor;
  bg = bgColor;

  Glib::RefPtr<Gtk::TextBuffer::Tag> refTagMatch = Gtk::TextBuffer::Tag::create();
  refTagMatch->property_foreground() = fg.c_str();

  Glib::RefPtr<Gtk::TextBuffer> refTextBuffer = m_TextView.get_buffer();
  Glib::RefPtr<Gtk::TextBuffer::TagTable> tagTable = refTextBuffer->get_tag_table();
  tagTable->add(refTagMatch);
  
  Gtk::TextBuffer::iterator iterStart = refTextBuffer->begin();
  Gtk::TextBuffer::iterator iterStop = refTextBuffer->end();
  refTextBuffer->apply_tag(refTagMatch,iterStart,iterStop);

  Gdk::Color bColor(bgColor);
  m_TextView.modify_base(Gtk::STATE_NORMAL, bColor);
  show_all_children();
}

CBAPITextWindow::~CBAPITextWindow() { }

void CBAPITextWindow::append_text(string text) {
  Glib::RefPtr<Gtk::TextBuffer> refTextBuffer = m_TextView.get_buffer();
  int last_line = refTextBuffer->get_line_count();
  Gtk::TextBuffer::iterator iter = refTextBuffer->get_iter_at_line(last_line);
  refTextBuffer->insert(iter, text);

  Glib::RefPtr<Gtk::TextBuffer::Tag> refTagMatch = Gtk::TextBuffer::Tag::create();
  refTagMatch->property_foreground() = fg.c_str();

  Glib::RefPtr<Gtk::TextBuffer::TagTable> tagTable = refTextBuffer->get_tag_table();
  tagTable->add(refTagMatch);
  
  Gtk::TextBuffer::iterator iterStart = refTextBuffer->begin();
  Gtk::TextBuffer::iterator iterStop = refTextBuffer->end();
  refTextBuffer->apply_tag(refTagMatch,iterStart,iterStop);

  set_adjustment();   
}


void CBAPITextWindow::prepend_text(string text) {
  Glib::RefPtr<Gtk::TextBuffer> refTextBuffer = m_TextView.get_buffer();
  Gtk::TextBuffer::iterator iter = refTextBuffer->get_iter_at_line(0);
  refTextBuffer->insert(iter, text);
 
}

void CBAPITextWindow::clear_text() {
  Glib::RefPtr<Gtk::TextBuffer> refTextBuffer = m_TextView.get_buffer();
  Gtk::TextBuffer::iterator iter = refTextBuffer->get_iter_at_offset(0);
  refTextBuffer->set_text("");
}


void CBAPITextWindow::set_adjustment() {
  Glib::RefPtr<Gtk::TextBuffer> refTextBuffer = m_TextView.get_buffer();
  int last_line = refTextBuffer->get_line_count();
  Gtk::TextBuffer::iterator iter = refTextBuffer->get_iter_at_line(last_line);
  m_TextView.scroll_to_iter(iter,0.0);
}
