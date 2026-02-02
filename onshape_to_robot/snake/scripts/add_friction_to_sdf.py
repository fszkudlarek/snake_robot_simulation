#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import sys

def add_friction_to_collision(collision, mu_value):
    """Add friction parameters to a collision element."""
    # Check if surface already exists
    surface = collision.find('surface')
    if surface is None:
        surface = ET.SubElement(collision, 'surface')
    
    # Check if friction already exists
    friction = surface.find('friction')
    if friction is None:
        friction = ET.SubElement(surface, 'friction')
    
    # Add ODE friction
    ode = friction.find('ode')
    if ode is None:
        ode = ET.SubElement(friction, 'ode')
    
    # Set mu and mu2
    mu = ode.find('mu')
    if mu is None:
        mu = ET.SubElement(ode, 'mu')
    mu.text = str(mu_value)
    
    mu2 = ode.find('mu2')
    if mu2 is None:
        mu2 = ET.SubElement(ode, 'mu2')
    mu2.text = str(mu_value)

def process_sdf(input_file, output_file):
    """Process SDF file and add friction to adhesive pads."""
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    # Find all links
    for link in root.findall('.//link'):
        link_name = link.get('name')
        
        if link_name and 'inner_adhesive_pad' in link_name:
            print(f"Processing {link_name} (inner pad - low friction μ=0.05)")
            # Find all collision elements in this link
            for collision in link.findall('collision'):
                add_friction_to_collision(collision, 0.05)
        
        elif link_name and 'outer_adhesive_pad' in link_name:
            print(f"Processing {link_name} (outer pad - high friction μ=0.9)")
            # Find all collision elements in this link
            for collision in link.findall('collision'):
                add_friction_to_collision(collision, 0.9)
    
    # Write the modified SDF
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"\nFriction parameters added! Output saved to: {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 add_friction_to_sdf.py input.sdf output.sdf")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    process_sdf(input_file, output_file)