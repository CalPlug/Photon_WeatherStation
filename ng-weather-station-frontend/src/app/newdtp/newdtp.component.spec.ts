import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { NewdtpComponent } from './newdtp.component';

describe('NewdtpComponent', () => {
  let component: NewdtpComponent;
  let fixture: ComponentFixture<NewdtpComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ NewdtpComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(NewdtpComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
