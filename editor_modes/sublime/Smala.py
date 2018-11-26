import sublime
import sublime_plugin
import os
import xml.etree.ElementTree as etree



#This plugin detects svg files in the content of the code
# and parse it to expose identified elements in completion

svg_ns = '{http://www.w3.org/2000/svg}'
svg_elements_with_id = []

def find_svg_regions(view):
		# find all svg files reference in the file
		pattern = "\"\w.*svg\"|\"\/\w.*svg\""
		regions =  view.find_all(pattern, sublime.IGNORECASE )
		return regions

def build_id_for_elem ( elem, prefix =""):
	if 'id' in elem.attrib:
		item = [elem.tag.replace(svg_ns, ''), prefix + elem.attrib['id']]
		if item not in svg_elements_with_id:
			svg_elements_with_id.append(item)
		
	if(elem.tag == svg_ns+'g') and ('id' in elem.attrib):
		for child in elem:
			build_id_for_elem(child, prefix + elem.attrib['id'] + ".")

def is_valid_path( path):
	return (path[-3:] == 'svg' and os.path.isfile(path))

def retrieve_identified_elements( view):
	regions = find_svg_regions(view)
	
	for region in regions :
		path = view.substr(region)
		path = path[1:-1]

		#build absolute path if relative given
		if(path[0]!= "/"):
			dirname = os.path.dirname(view.file_name())
			dirname = os.path.dirname(dirname)
			path = os.path.join(dirname, path)

		if not is_valid_path(path):
			raise NameError('file %s is not an svg or does not exists', path)
			
		#print('extracting identified elements for ' + path)
			
		tree = etree.parse(path)
		root = tree.getroot()
		if root.tag != svg_ns + 'svg':
			raise TypeError('file %s does not seem to be a valid SVG file', path)

		for child in root:
			build_id_for_elem(child)


class SmalaAutocomplete(sublime_plugin.EventListener):
	def on_query_completions(self, view, prefix, locations):
		sugs = []
		print(svg_elements_with_id)
		retrieve_identified_elements(view)
		for elem in svg_elements_with_id:
			sugs.append([elem[1] + '\t' + elem[0], elem[1]])

		return sugs