import xml.etree.ElementTree as ET

def add_suffix_to_urdf(input_file, output_file, suffix):
    # Parse the URDF file
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    # Define the tags that need to be suffixed
    tags_to_suffix = ['link', 'joint']

    # Function to add suffix to a name
    def add_suffix(name, suffix):
        return f"{name}{suffix}"

    # Iterate through all the elements and add suffix to links and joints
    for element in root.iter():
        if element.tag in tags_to_suffix:
            if 'name' in element.attrib:
                element.attrib['name'] = add_suffix(element.attrib['name'], suffix)

        # Also need to update child elements that refer to these names
        if element.tag == 'parent' or element.tag == 'child':
            if 'link' in element.attrib:
                element.attrib['link'] = add_suffix(element.attrib['link'], suffix)
        if element.tag == 'origin' or element.tag == 'geometry':
            if 'reference' in element.attrib:
                element.attrib['reference'] = add_suffix(element.attrib['reference'], suffix)

    # Write the modified URDF to a new file
    tree.write(output_file, encoding='utf-8', xml_declaration=True)

# Example usage
input_urdf_file = 'URDF_files/spirit_1_truss.urdf'  # Replace with your input URDF file path
output_urdf_file = 'URDF_files/spirit_1_truss_0.urdf'  # Replace with your desired output URDF file path
suffix = '_spirit0'  # Replace with your desired suffix

add_suffix_to_urdf(input_urdf_file, output_urdf_file, suffix)
