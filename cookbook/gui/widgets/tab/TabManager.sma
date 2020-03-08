/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2018)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *    	Nicolas Saporito  <nicolas.saporito@enac.fr>
 *
 */
use core
use exec_env
use base
use display
use gui


_action_
reorder_tabs_action (Process c)
%{
	Process *my_tab_manager = (Process*) get_native_user_data (c);
	List *tabs_list = (List*) my_tab_manager->find_component ("tabs");
	char spec[16];

	// The old selected tab is at the end of the tabs list,
	// get it, tell it to unselect itself...
	sprintf (spec, "%d", tabs_list->size());
	Process *old_selected_tab = (Process*) tabs_list->find_component (spec);
	Spike *unselect = (Spike*) old_selected_tab->find_component ("unselect");
	unselect->activate ();
	// get its original index...
	int old_selected_index = ((IntProperty*) old_selected_tab->find_component ("index"))->get_value ();
	sprintf (spec, "<%d", old_selected_index);
	// ...and move it back to its original place
	tabs_list->remove_child (old_selected_tab);
	tabs_list->insert (old_selected_tab, spec);

	// Find the new selected tab and move it to the end of the tabs list
	int new_selected_index = ((IntProperty*) my_tab_manager->find_component ("selected_index"))->get_value ();
	for (Process *tab : tabs_list->children()) {
		int tab_index = ((IntProperty*) tab->find_component ("index"))->get_value ();
		if (tab_index == new_selected_index) {
			tabs_list->remove_child (tab);
			tabs_list->add_child (tab, "");
			break;
		}
	}
%}


_define_
TabManager () {
	Double selected_color (40)
	Double unselected_color (65)
	Double selected_font_color (255)
	Double unselected_font_color (150)
	Double unselected_border_color (55)
	Double selected_border_color (30)

	List tabs

	Int selected_index (0)
	NativeAction reorder_tabs (reorder_tabs_action, this, 1)
	selected_index -> reorder_tabs
}
