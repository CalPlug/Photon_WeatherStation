import { Component, OnInit } from '@angular/core';

@Component({
  selector: 'app-about',
  templateUrl: '../stdpage/stdpage.component.html',
  styleUrls: ['../stdpage/stdpage.component.css']
})
export class AboutComponent implements OnInit {

  title:string = "About";
  content:string = `
      <p>This Node/Angular-based web applications allows for the viewing of weather patterns recorded by the Weather Station and stored into a database.</p>
    `;

  constructor() { }

  ngOnInit() {
  }

}
